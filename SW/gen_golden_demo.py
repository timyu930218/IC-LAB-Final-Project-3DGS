import numpy as np
import os
import math
import struct
import imageio
from PIL import Image

# ==========================================
# Configuration
# ==========================================
NUM_PATTERNS = 64
OUTPUT_DIR = "golden_demo"

# Set random seed for reproducibility
# np.random.seed(0) removed for single executable request

# Subdirectories for 8 Stages
DIRS = {
    "input": os.path.join(OUTPUT_DIR, "input"),
    "stage1": os.path.join(OUTPUT_DIR, "stage1_rot"),
    "stage2": os.path.join(OUTPUT_DIR, "stage2_cov3d"),
    "stage3": os.path.join(OUTPUT_DIR, "stage3_cam_pos"),
    "stage4": os.path.join(OUTPUT_DIR, "stage4_jacobian"),
    "stage5": os.path.join(OUTPUT_DIR, "stage5_cov2d"),
    "stage6": os.path.join(OUTPUT_DIR, "stage6_conic"),
    "stage_bbox": os.path.join(OUTPUT_DIR, "stage_bbox"),
    "stage7": os.path.join(OUTPUT_DIR, "stage7_sort"),
    "stage8": os.path.join(OUTPUT_DIR, "stage8_render"),
    "stage_raster_param": os.path.join(OUTPUT_DIR, "stage_raster_param"),
    "visual": os.path.join(OUTPUT_DIR, "visual_debug")
}

for d in DIRS.values():
    os.makedirs(d, exist_ok=True)

# ==========================================
# Quantization Helpers
# ==========================================

class Quantizer:
    def __init__(self, int_bits, frac_bits):
        self.int_bits = int_bits
        self.frac_bits = frac_bits
        self.total_bits = int_bits + frac_bits 
        self.min_val = -(1 << (int_bits - 1))
        self.max_val = (1 << (int_bits - 1)) - (2 ** -frac_bits)
        self.scale = 2 ** frac_bits

    def quantize(self, x):
        if x is None: return None
        if isinstance(x, np.ndarray):
            return np.vectorize(self.quantize)(x)
        
        # Round Half Up (Standard Hardware Rounding)
        val = math.floor(x * self.scale + 0.5)
        
        min_int = -(1 << (self.total_bits - 1))
        max_int = (1 << (self.total_bits - 1)) - 1
        
        if val > max_int: val = max_int
        if val < min_int: val = min_int
        
        return val / self.scale

    def quantize_trunc(self, x):
        if x is None: return None
        if isinstance(x, np.ndarray):
            return np.vectorize(self.quantize_trunc)(x)
            
        val = int(x * self.scale)
        
        min_int = -(1 << (self.total_bits - 1))
        max_int = (1 << (self.total_bits - 1)) - 1
        
        if val > max_int: val = max_int
        if val < min_int: val = min_int
        
        return val / self.scale

    def to_hex(self, x):
        val = int(math.floor(x * self.scale + 0.5))
        mask = (1 << self.total_bits) - 1
        val = val & mask
        hex_digits = (self.total_bits + 3) // 4
        return f"{val:0{hex_digits}x}"

# Define Quantizers
Q_INPUT = Quantizer(12, 20) # 32-bit
Q_MAIN = Quantizer(10, 6)   # 16-bit
Q_CONIC = Quantizer(4, 12)  # 16-bit
Q_DET_INV = Quantizer(14, 18) # 32-bit intermediate
Q_COLOR = Quantizer(8, 8)   # 16-bit

def to_hex_16(q, x):
    val = int(math.floor(x * q.scale + 0.5))
    mask = (1 << 16) - 1
    val = val & mask
    return f"{val:04x}"

# ==========================================
# Math Helpers (Quantized)
# ==========================================

def quat_to_rotmat(q):
    w, x, y, z = q
    # Operations in Q_MAIN
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])
    return Q_MAIN.quantize(R)

def compute_scale_mat(scale):
    S = np.diag(scale)
    return Q_MAIN.quantize(S)

def compute_cov3d(scale_mat, rot_mat):
    M = rot_mat @ scale_mat
    M = Q_MAIN.quantize(M)
    Sigma = M @ M.T
    return Q_MAIN.quantize(Sigma)

def compute_cam_pos_hw(xyz, view_matrix):
    # Hardware-Accurate Integer Implementation
    # xyz and view_matrix are already quantized floats.
    # Convert to integer representation (Q10.6)
    
    def to_int(val_float):
        return int(math.floor(val_float * 64 + 0.5))

    x_int = to_int(xyz[0])
    y_int = to_int(xyz[1])
    z_int = to_int(xyz[2])
    
    # view_matrix is 4x4. We need the rotation part and translation part.
    # Hardware does row-wise dot product.
    
    t_int = np.zeros(3, dtype=int)
    
    for i in range(3):
        row_0 = to_int(view_matrix[i, 0])
        row_1 = to_int(view_matrix[i, 1])
        row_2 = to_int(view_matrix[i, 2])
        row_3 = to_int(view_matrix[i, 3]) # Translation
        
        # Accumulator: 34 bits in HW.
        # acc = x*v0 + y*v1 + z*v2 + (v3 << 6)
        
        prod_0 = x_int * row_0 # Q20.12
        prod_1 = y_int * row_1
        prod_2 = z_int * row_2
        
        term_3 = row_3 << 6    # Align Q10.6 -> Q16.12 (Wait, Q10.6 * 64 = Q16.12 effectively)
        
        acc = prod_0 + prod_1 + prod_2 + term_3
        
        # Output Q10.6: Round and Shift
        # (acc + 32) >> 6
        val = (acc + 32) >> 6
        t_int[i] = val
        
    # Return as floats for compatibility with rest of pipeline, but derived from int
    return t_int / 64.0, t_int

def compute_jacobian(t, proj_matrix, width, height):
    x, y, z = t
    fx = proj_matrix[0,0] * width / 2
    fy = proj_matrix[1,1] * height / 2
    
    if z == 0: z = 0.0001 # Prevent div by zero in Python (HW might handle differently or produce garbage)
    
    j00 = Q_MAIN.quantize_trunc(fx/z)
    j02 = Q_MAIN.quantize_trunc(-(fx*x)/(z**2))
    j11 = Q_MAIN.quantize_trunc(-fy/z)
    j12 = Q_MAIN.quantize_trunc((fy*y)/(z**2))
    
    J = np.array([
        [j00, 0, j02],
        [0, j11, j12]
    ])
    return J

def compute_cov2d(cov3d, view_matrix, jacobian):
    W = view_matrix[:3, :3]
    Sigma_cam = W @ cov3d @ W.T
    Sigma_cam = Q_MAIN.quantize(Sigma_cam)
    
    cov2d = jacobian @ Sigma_cam @ jacobian.T
    cov2d = Q_MAIN.quantize(cov2d)
    
    # Low pass
    offset = 19.0 / 64.0
    cov2d[0,0] += offset
    cov2d[1,1] += offset
    
    cov2d_clamped = Q_MAIN.quantize(cov2d)
    
    # For BBox, we need the "Raw" integer value
    def quantize_no_clamp(arr, fract_bits=6):
        scale = (1 << fract_bits)
        return np.floor(arr * scale + 0.5) / scale

    cov2d_unclamped = quantize_no_clamp(cov2d)
    
    return cov2d_clamped, cov2d_unclamped

def compute_conic(cov2d):
    # Hardware-Accurate Integer Implementation matching Stage6_Conic.v
    sigma00 = to_signed(to_fixed(cov2d[0,0], 64), 16)
    sigma01 = to_signed(to_fixed(cov2d[0,1], 64), 16)
    sigma11 = to_signed(to_fixed(cov2d[1,1], 64), 16)
    
    mult1 = to_signed(sigma00 * sigma11, 32)
    mult2 = to_signed(sigma01 * sigma01, 32)
    
    det_q = to_signed(mult1 - mult2, 32)
    
    if det_q < 0:
        sign = -1
        det_abs = -det_q
    else:
        sign = 1
        det_abs = det_q
        
    if det_abs == 0: det_abs = 1
    
    numerator = (1 << 35) + (det_abs >> 1)
    det_inv_abs = numerator // det_abs
    
    if sign == -1:
        det_inv_q = -det_inv_abs
    else:
        det_inv_q = det_inv_abs
        
    term_a = sigma11 * det_inv_q
    conic_a_int = (term_a + (1 << 16)) >> 17
    
    term_b = (-sigma01) * det_inv_q
    conic_b_int = (term_b + (1 << 16)) >> 17
    
    
    term_c = sigma00 * det_inv_q
    conic_c_int = (term_c + (1 << 16)) >> 17
    
    # 2024-12-17 Fix: Saturate Conic values to 16-bit signed to prevent wrapping artifacts
    # When det is small, det_inv is large, leading to huge Conic values.
    # Wrapping them (to_signed) creates negative Conic values (Inverted Gaussians -> Giant Blocks).
    # We must clamp to range [-32767, 32767] (or similar safe range).
    
    def saturate_16bit(val):
        if val > 32767: return 32767
        if val < -32768: return -32768
        return val

    conic_a_int = saturate_16bit(conic_a_int)
    conic_b_int = saturate_16bit(conic_b_int)
    conic_c_int = saturate_16bit(conic_c_int)
    
    # Convert back to float for output compatibility (Q4.12 -> float)
    return np.array([conic_a_int, conic_b_int, conic_c_int]) / 4096.0

def generate_sqrt_lut():
    lut = []
    # 16-bit Q10.6
    TOTAL_BITS = 16
    FRAC_BITS = 6
    SCALE = 64
    NUM_ENTRIES = 65536
    
    for i in range(NUM_ENTRIES):
        # Interpret i as signed 16-bit integer
        val_s = i
        if val_s >= 32768:
            val_s -= 65536
            
        # Convert to float
        val_float = val_s / float(SCALE)
        
        # Apply Logic: max(0.05, val)
        effective_val = max(0.05, val_float)
        
        # Compute 3 * sqrt(x)
        res = 3.0 * math.sqrt(effective_val)
        
        # Quantize Output (Q10.6 Round Half Up)
        res_quant = int(math.floor(res * SCALE + 0.5))
        
        # Clamp to 16-bit range (Signed)
        if res_quant > 32767: res_quant = 32767
        if res_quant < -32768: res_quant = -32768
        
        lut.append(res_quant)
    return lut

SQRT_LUT = generate_sqrt_lut()

def emulate_sqrt_lut(val_int):
    # Hardware Logic: If input is negative, hardware typically handles it (reg <= 0).
    # LUT logic takes 16-bit input.
    if val_int < 0: val_int = 0
    if val_int >= 65536: val_int = 65535 # Clamp magnitude
    
    # Use internal LUT
    return SQRT_LUT[val_int]

def compute_bbox(center, cov2d, width, height):
    # 128 Mode Logic matching Hardware Stage5_Conv2D with Split Shift Bug/Feature
    
    # 1. Convert cov2d to Q10.6 Int
    cov00_int = int(math.floor(cov2d[0,0] * 64 + 0.5))
    cov11_int = int(math.floor(cov2d[1,1] * 64 + 0.5))
    
    # 2. Split Shift Logic
    # Hardware: Sqrt_In = ((Raw + 2048) >> 14) + (19 >> 2)
    # Python: cov_int = Raw + 19 (approx, if we used 19/64.0 offset exactly)
    # We need to extract Raw or simulate the split.
    # Since we updated compute_cov2d to use 19/64.0, cov00_int is exactly Raw + 19.
    # Raw = cov00_int - 19
    
    raw_x = cov00_int - 19
    raw_y = cov11_int - 19
    
    # Hardware Logic: sqrt_in = (Raw_Rounded >> 2) + 4
    # Raw is Q10.6 int (scaled by 64).
    
    sqrt_in_x = (raw_x >> 2) + 4
    sqrt_in_y = (raw_y >> 2) + 4
    
    # 3. Emulate Sqrt LUT (Input Q10.6 -> Output Q10.6)
    # LUT computes: y = round(3 * sqrt(max(0.05, x)))
    
    def emulate_sqrt_lut(val_int):
        if val_int < 0: val_int = 0
        val_float = val_int / 64.0
        # LUT bakes in max(0.05, val)
        rad = 3.0 * math.sqrt(max(0.05, val_float))
        return int(math.floor(rad * 64 + 0.5))

    lut_out_x = emulate_sqrt_lut(sqrt_in_x)
    lut_out_y = emulate_sqrt_lut(sqrt_in_y)
    
    # 4. Final Shift Left by 1 (Force Even)
    rad_x_int = lut_out_x << 1
    rad_y_int = lut_out_y << 1
    
    rad_x = rad_x_int / 64.0
    rad_y = rad_y_int / 64.0
    
    min_x = max(0, int(center[0] - rad_x))
    max_x = min(width - 1, int(center[0] + rad_x))
    min_y = max(0, int(center[1] - rad_y))
    max_y = min(height - 1, int(center[1] + rad_y))
    
    return min_x, max_x, min_y, max_y, rad_x, rad_y

# ==========================================
# Look-Up Table Loader
# ==========================================

def generate_exp_lut():
    lut = []
    for i in range(2048):
        x = i / 256.0
        val = math.exp(-x)
        q_val = int(math.floor(val * 256 + 0.5))
        if q_val > 65535: q_val = 65535
        lut.append(q_val)
    return lut

EXP_LUT = generate_exp_lut()

def hardware_exp(power_int):
    # load_exp_lut() removed
    p_val = power_int
    if p_val > 32767: p_val -= 65536
    if p_val > 0: return 0.0
    p_abs = abs(p_val)
    lut_addr_full = p_abs >> 4
    lut_addr = 2047 if lut_addr_full > 2047 else (lut_addr_full & 0x7FF)
    val_q88 = EXP_LUT[lut_addr]
    return val_q88 / 256.0

def to_signed(val, bits):
    mask = (1 << bits) - 1
    val_masked = val & mask
    if val_masked & (1 << (bits - 1)):
        return val_masked - (1 << bits)
    return val_masked

def to_fixed(val, scale):
    return int(math.floor(val * scale + 0.5))

def evaluate_gaussian(s_idx, pixel_x, pixel_y, screenspace_center, conic, opacity):
    center_x_int = to_signed(to_fixed(screenspace_center[0], 64), 16)
    center_y_int = to_signed(to_fixed(screenspace_center[1], 64), 16)
    
    conic_a_int = to_signed(to_fixed(conic[0], 4096), 16)
    conic_b_int = to_signed(to_fixed(conic[1], 4096), 16)
    conic_c_int = to_signed(to_fixed(conic[2], 4096), 16)
    
    x_shifted = pixel_x << 6
    y_shifted = pixel_y << 6
    
    delta_x_int = to_signed(x_shifted - center_x_int, 16)
    delta_y_int = to_signed(y_shifted - center_y_int, 16)
    
    dx_sq = to_signed(delta_x_int * delta_x_int, 32)
    dy_sq = to_signed(delta_y_int * delta_y_int, 32)
    dx_dy = to_signed(delta_x_int * delta_y_int, 32)
    
    term_a = to_signed(conic_a_int * dx_sq, 48)
    term_b = to_signed(conic_b_int * dx_dy, 48)
    term_c = to_signed(conic_c_int * dy_sq, 48)
    
    term_b_shifted = to_signed(term_b << 1, 48)
    sum_terms = to_signed(term_a + term_b_shifted + term_c, 48)
    sum_shifted = sum_terms >> 1 
    power_temp = to_signed(-sum_shifted, 48)
    power_shifted = to_signed(power_temp + 2048, 48) >> 12
    
    MIN_VAL = -32768
    if power_shifted < MIN_VAL:
        P = MIN_VAL 
    else:
        if power_shifted > 32767:
            P = 32767
        else:
            P = to_signed(power_shifted, 16)
            
    # DEBUG P Value for Pattern 51 Pixel 15
    if s_idx == 51 and pixel_x == 15 and pixel_y == 0:
        print(f"DEBUG_SW_EVAL: ID={s_idx} P={P} PowerShifted={power_shifted} Opacity={opacity}")
        print(f"  DeltaX={delta_x_int} DeltaY={delta_y_int}")
        print(f"  DxSq={dx_sq} DySq={dy_sq} DxDy={dx_dy}")
        print(f"  Conic A={conic_a_int} B={conic_b_int} C={conic_c_int}")
        print(f"  Term A={term_a} B={term_b} C={term_c}")
        print(f"  Sum={sum_terms} PowerTemp={power_temp}")

    if P > 0: return 0.0
        
    val_float = hardware_exp(P)
    val_q88_int = int(val_float * 256 + 0.5)
    
    opacity_int = to_signed(to_fixed(opacity, 64), 16)
    alpha_prod = to_signed(opacity_int * val_q88_int, 32)
    alpha_rounded = (alpha_prod + 32) >> 6
    alpha_rounded = to_signed(alpha_rounded, 32)
    
    if alpha_rounded <= 1: alpha_final_int = 0
    elif alpha_rounded > 253: alpha_final_int = 253
    else: alpha_final_int = alpha_rounded

    if alpha_rounded <= 1: alpha_final_int = 0
    elif alpha_rounded > 253: alpha_final_int = 253
    else: alpha_final_int = alpha_rounded

    return alpha_final_int / 256.0

def simulate_accumulation_hardware_flow(gaussians_sorted, width, height, s_idx):
    sram = np.zeros((height, width, 4), dtype=np.int32)
    sram[:, :, 0] = 256 
    
    # DEBUG: Create debug directory for pattern 50
    debug_dir = os.path.join(OUTPUT_DIR, "debug_demo_50")
    if s_idx == 50:
        os.makedirs(debug_dir, exist_ok=True)

    for i, g in enumerate(gaussians_sorted):
        min_x, max_x, min_y, max_y = g['bbox'][:4]
        # In gen_golden.py, we typically tracked 'idx'. Let's see if 'g' has it.
        # Check 'generate_random_gaussians' or where 'gaussians_sorted' comes from.
        # It comes from 'sort_gaussians_bitonic' which returns list of dicts.
        # The dicts are created in the main loop.
        # In gen_golden_demo.py:
        # gaussian_objects.append({ 'idx': i, ... })
        # So 'g' SHOULD have 'idx'.
        
        gid = g.get('idx', -1)
        
        # DEBUG Specific Sorted Index 1
        if s_idx == 51 and i == 1:
             print(f"DEBUG_SW_SORTED_IDX1: Index={i} OrigID={gid} Center={g['screen_pos']} BBox=[{min_x},{max_x}]x[{min_y},{max_y}]")
             # Re-calculate integer conic for display to match HW comparison
             # (This logic is roughly what evaluate_gaussian does, effectively)
             # But here we just print floats
             print(f"  Conic Float: {g['conic']}")
             
             # Calculate and print Integer Conics as per HW
             conic_a_int = int(g['conic'][0] * 4096)
             conic_b_int = int(g['conic'][1] * 4096)
             conic_c_int = int(g['conic'][2] * 4096)
             print(f"  Conic Int: A={conic_a_int} B={conic_b_int} C={conic_c_int}")
             print(f"  Color: {g['color']}")

        # DEBUG BBox for Pixel 15 coverage
        if s_idx == 51:
             if min_y <= 0 <= max_y and min_x <= 15 <= max_x:
                 print(f"DEBUG_SW_BBOX: SortedIdx={i} OrigID={gid} Center={g['screen_pos']} BBox=[{min_x},{max_x}]x[{min_y},{max_y}]")

        for y in range(min_y, max_y + 1):
            if y < 0 or y >= height: continue
            for x in range(min_x, max_x + 1):
                if x < 0 or x >= width: continue
                
                alpha_float = evaluate_gaussian(s_idx, x, y, g['screen_pos'], g['conic'], g['opacity'])
                alpha_q8 = int(alpha_float * 256 + 0.5)
                if alpha_q8 <= 0: continue
                
                t_old = sram[y, x, 0]
                r_old = sram[y, x, 1]
                g_old = sram[y, x, 2]
                b_old = sram[y, x, 3]
                
                if t_old == 0: continue
                
                weight_q8 = (alpha_q8 * t_old) >> 8
                
                c_r_int = int(g['color'][0] * 64 + 0.5)
                c_g_int = int(g['color'][1] * 64 + 0.5)
                c_b_int = int(g['color'][2] * 64 + 0.5)
                
                # DEBUG PRINT for Pattern 51, Pixel (15,0)
                if s_idx == 51 and x == 15 and y == 0:
                    delta_x_val = x*64 + 32 - g['screen_pos'][0]*64
                    delta_y_val = y*64 + 32 - g['screen_pos'][1]*64
                    
                    # Recalculate power for debug print 
                    conic_Arr = g['conic']
                    # This logic duplicates evaluate_gaussian but allows printing intermediates if needed relative to HW
                    # But evaluate_gaussian returns alpha_float. 
                    
                    print(f"DEBUG_ACCUM_51: ID={g['id']} Pos=({x},{y}) AlphaQ8={alpha_q8} T_old={t_old} W_q8={weight_q8}")
                    print(f"  screen_pos=({g['screen_pos'][0]}, {g['screen_pos'][1]})")
                    
                prod_r = c_r_int * weight_q8 
                prod_g = c_g_int * weight_q8 
                prod_b = c_b_int * weight_q8 
                
                sum_r = prod_r + (r_old << 8)
                sum_g = prod_g + (g_old << 8)
                sum_b = prod_b + (b_old << 8)
                
                mid_r = (sum_r + 32) >> 6
                mid_g = (sum_g + 32) >> 6
                mid_b = (sum_b + 32) >> 6
                
                r_new = (mid_r + 2) >> 2
                g_new = (mid_g + 2) >> 2
                b_new = (mid_b + 2) >> 2
                
                inv_alpha = 256 - alpha_q8
                t_new = (t_old * inv_alpha) >> 8
                
                sram[y, x, 0] = t_new
                sram[y, x, 1] = r_new
                sram[y, x, 2] = g_new
                sram[y, x, 3] = b_new
        
        # DEBUG: Save intermediate image for Pattern 50
        if s_idx == 50:
            debug_fname = os.path.join(debug_dir, f"golden_image_50_{i:02d}.dat")
            with open(debug_fname, "w") as f_debug:
                flat_sram = sram.reshape(-1, 4)
                # sram format: T, R, G, B (each 16-bit effectively, but stored as int32 here)
                # Output format: 64-bit word containing [T, R, G, B] each 16-bit
                for pixel in flat_sram:
                    t, r, g, b = int(pixel[0]), int(pixel[1]), int(pixel[2]), int(pixel[3])
                    val = (t << 48) | (r << 32) | (g << 16) | b
                    f_debug.write(f"{val:016x}\n")

                
    final_canvas = np.zeros((height, width, 3))
    final_canvas[:, :, 0] = sram[:, :, 1] / 64.0
    final_canvas[:, :, 1] = sram[:, :, 2] / 64.0
    final_canvas[:, :, 2] = sram[:, :, 3] / 64.0
    
    return final_canvas, sram

# ==========================================
# Scene Content Generators
# ==========================================

def create_letter_scene(char):
    gaussians = []
    opacity = 0.9
    scale = np.array([0.1, 0.1, 0.1])
    rot = np.array([1.0, 0.0, 0.0, 0.0])
    points = []
    color = np.array([1.0, 1.0, 1.0])
    
    if char == 'I':
        color = np.array([1.0, 0.2, 0.2])
        for i in range(64):
            t = i / 63.0
            y = -1.5 + 3.0 * t
            points.append(np.array([0.0, y, 0.0]))
    elif char == 'C':
        color = np.array([1.0, 0.6, 0.0])
        num_points = 64
        radius = 1.0
        for i in range(num_points):
            t = i / (num_points - 1)
            start_angle = math.pi / 4
            end_angle = 7 * math.pi / 4
            angle = start_angle + (end_angle - start_angle) * t
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            points.append(np.array([x, y, 0.0]))
    elif char == 'L':
        color = np.array([1.0, 1.0, 0.0])
        for i in range(40):
            t = i / 39.0
            y = -1.5 + 3.0 * t
            points.append(np.array([-0.8, y, 0.0]))
        for i in range(24):
            t = i / 23.0
            x = -0.8 + 1.6 * t
            points.append(np.array([x, -1.5, 0.0]))
    elif char == 'A':
        color = np.array([0.2, 1.0, 0.2])
        for i in range(25):
            t = i / 24.0
            x = -1.0 + 1.0 * t
            y = -1.5 + 3.0 * t
            points.append(np.array([x, y, 0.0]))
        for i in range(25):
            t = i / 24.0
            x = 0.0 + 1.0 * t
            y = 1.5 - 3.0 * t
            points.append(np.array([x, y, 0.0]))
        for i in range(14):
            t = i / 13.0
            x = -0.5 + 1.0 * t
            points.append(np.array([x, 0.0, 0.0]))
    elif char == 'B':
        color = np.array([0.2, 0.2, 1.0])
        for i in range(24):
            t = i / 23.0
            y = -1.5 + 3.0 * t
            points.append(np.array([-0.8, y, 0.0]))
        for i in range(20):
            t = i / 19.0
            angle = -math.pi/2 + math.pi * t
            x = -0.8 + 0.8 * math.cos(angle)
            y = 0.75 + 0.75 * math.sin(angle)
            points.append(np.array([x, y, 0.0]))
        for i in range(20):
            t = i / 19.0
            angle = -math.pi/2 + math.pi * t
            x = -0.8 + 0.8 * math.cos(angle)
            y = -0.75 + 0.75 * math.sin(angle)
            points.append(np.array([x, y, 0.0]))
            
    # Ensure exactly 64 points if not already
    while len(points) < 64:
        points.append(points[-1])
    if len(points) > 64:
        points = points[:64]
            
    for i, p in enumerate(points):
        gaussians.append({
            'pos': p, 'scale': scale, 'rot': rot, 'opacity': opacity, 'color': color, 'id': i
        })
    return gaussians

def create_cornell_scene():
    gaussians = []
    room_size = 4.0
    # Fixed setup for 64 Gaussians matching reference
    
    # Walls (45)
    step = room_size * 2 / 3.0
    pts = [-0.7 * room_size, 0.0, 0.7 * room_size]
    flat_scale = 0.05
    cover_scale = room_size / 1.2
    
    # Left (Red)
    c_red = np.array([1.0, 0.2, 0.2])
    for y in pts:
        for z in pts:
            gaussians.append({'pos': np.array([-room_size, y, z]), 'scale': np.array([flat_scale, cover_scale, cover_scale]), 'rot': np.array([1.0, 0.0, 0.0, 0.0]), 'opacity': 0.9, 'color': c_red})
            
    # Right (Green)
    c_green = np.array([0.2, 1.0, 0.2])
    for y in pts:
        for z in pts:
            gaussians.append({'pos': np.array([room_size, y, z]), 'scale': np.array([flat_scale, cover_scale, cover_scale]), 'rot': np.array([1.0, 0.0, 0.0, 0.0]), 'opacity': 0.9, 'color': c_green})
            
    # Back (White)
    c_white = np.array([0.9, 0.9, 0.9])
    for x in pts:
        for y in pts:
             gaussians.append({'pos': np.array([x, y, -room_size]), 'scale': np.array([cover_scale, cover_scale, flat_scale]), 'rot': np.array([1.0, 0.0, 0.0, 0.0]), 'opacity': 0.9, 'color': c_white})
             
    # Floor (White)
    for x in pts:
        for z in pts:
            gaussians.append({'pos': np.array([x, -room_size, z]), 'scale': np.array([cover_scale, flat_scale, cover_scale]), 'rot': np.array([1.0, 0.0, 0.0, 0.0]), 'opacity': 0.9, 'color': c_white})
            
    # Ceiling (White)
    for x in pts:
        for z in pts:
            gaussians.append({'pos': np.array([x, room_size, z]), 'scale': np.array([cover_scale, flat_scale, cover_scale]), 'rot': np.array([1.0, 0.0, 0.0, 0.0]), 'opacity': 0.9, 'color': c_white})
            
    # Light (1)
    gaussians.append({'pos': np.array([0.0, room_size - 0.2, 0.0]), 'scale': np.array([0.5, 0.1, 0.5]), 'rot': np.array([1.0, 0.0, 0.0, 0.0]), 'opacity': 0.9, 'color': np.array([1.0, 1.0, 1.0])})
    
    # Box (18)
    b_pts_x = [-0.3, 0.0, 0.3]
    b_pts_y = [-0.3, 0.0, 0.3]
    b_pts_z = [-0.2, 0.2]
    offset = np.array([0.5, -room_size + 0.6, 0.0])
    c_blue = np.array([0.2, 0.4, 0.8])
    
    for x in b_pts_x:
        for y in b_pts_y:
            for z in b_pts_z:
                gaussians.append({'pos': np.array([x, y, z]) + offset, 'scale': np.array([0.15, 0.15, 0.15]), 'rot': np.array([0.924, 0.0, 0.383, 0.0]), 'opacity': 0.9, 'color': c_blue})
                
    # Add IDs
    for i, g in enumerate(gaussians):
        g['id'] = i
        
    return gaussians

# ==========================================
# Main Generation Loop
# ==========================================

def generate_data():
    print(f"Generating {NUM_PATTERNS} demo scenes...")
    
    width, height = 128, 128
    
    # Projection Matrix
    fx_target = 111.0
    fy_target = 111.0
    
    p00 = fx_target / (width / 2.0)
    p11 = fy_target / (height / 2.0)
    
    proj_matrix = np.array([
        [p00, 0, 0, 0],
        [0, p11, 0, 0],
        [0, 0, 1, -0.2],
        [0, 0, 1, 0]
    ])
    proj_matrix = Q_MAIN.quantize(proj_matrix)
    
    frames_iclab = []
    frames_cornell = []
    
    for s in range(NUM_PATTERNS):
    # for s in range(51, 52): # Fast Debug 51
        print(f"DEBUG: Processing SCENE {s}...")
        
        # Determine Scene
        if s < 50:
            # ICLAB
            scene_type = 'ICLAB'
            letter_idx = s // 10
            char = ['I', 'C', 'L', 'A', 'B'][letter_idx]
            gaussians_data = create_letter_scene(char)
            
            # Orbit View
            # 10 frames per letter, full circle (360 deg) or Partial?
            # Let's do 360 deg over 10 frames = 36 deg per frame
            frame_in_letter = s % 10
            angle = 2 * math.pi * frame_in_letter / 10.0
            radius = 4.0
            eye_x = radius * math.sin(angle)
            eye_z = radius * math.cos(angle)
            eye = np.array([eye_x, 1.0, eye_z])
            target = np.array([0.0, 0.0, 0.0])
            
        else:
            # Cornell Box (Patterns 50-63, remapped from original 58-71)
            # Original Logic: t = (orig_idx - 50) / 99.0
            # New Logic: orig_idx = s + 8
            # So t = (s + 8 - 50) / 99.0
            
            scene_type = 'CORNELL'
            gaussians_data = create_cornell_scene()
            
            # Animation: Camera moves
            t = (s + 8 - 50) / 99.0
            
            # Use same "Wobble" path as before
            dist_t = math.sin(t * math.pi) # 0 -> 1 -> 0
            angle_deg = -30 + 60 * math.sin(2 * math.pi * t) # One full sway?
            angle = angle_deg * math.pi / 180
            
            dist_t = math.sin(4 * math.pi * t) # Front back wobble
            radius = 9.0 + 1.5 * dist_t # 7.5 to 10.5 likely
            
            eye_x = radius * math.sin(angle)
            eye_z = radius * math.cos(angle)
            eye = np.array([eye_x, 0.0, eye_z])
            target = np.array([0.0, 0.0, 0.0])
            
        num_gaussians = len(gaussians_data)
            
        # View Matrix
        up = np.array([0.0, 1.0, 0.0])
        z_axis = target - eye
        z_axis /= np.linalg.norm(z_axis)
        x_axis = np.cross(up, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        
        view_matrix = np.eye(4)
        view_matrix[:3, 0] = x_axis
        view_matrix[:3, 1] = y_axis
        view_matrix[:3, 2] = z_axis
        view_matrix[:3, 3] = -np.dot(np.array([x_axis, y_axis, z_axis]), eye)
        view_matrix = Q_MAIN.quantize(view_matrix)
        
        # Open Files
        f_in_gauss = open(os.path.join(DIRS["input"], f"input_gaussian_{s:02d}.dat"), "w")
        f_in_view = open(os.path.join(DIRS["input"], f"input_view_{s:02d}.dat"), "w")
        
        f_rot = open(os.path.join(DIRS["stage1"], f"golden_rot_mat_{s:02d}.dat"), "w")
        f_cov3d = open(os.path.join(DIRS["stage2"], f"golden_cov3d_{s:02d}.dat"), "w")
        f_cam = open(os.path.join(DIRS["stage3"], f"golden_cam_pos_{s:02d}.dat"), "w")
        f_jac = open(os.path.join(DIRS["stage4"], f"golden_jacobian_{s:02d}.dat"), "w")
        f_cov2d = open(os.path.join(DIRS["stage5"], f"golden_cov2d_{s:02d}.dat"), "w")
        f_conic = open(os.path.join(DIRS["stage6"], f"golden_conic_{s:02d}.dat"), "w")
        f_bbox = open(os.path.join(DIRS["stage_bbox"], f"golden_bbox_{s:02d}.dat"), "w")
        f_sort = open(os.path.join(DIRS["stage7"], f"golden_sort_{s:02d}.dat"), "w")
        f_param = open(os.path.join(DIRS["stage_raster_param"], f"golden_raster_param_{s:02d}.dat"), "w")
        f_img = open(os.path.join(DIRS["stage8"], f"golden_image_{s:02d}.dat"), "w")
        
        # Write View
        view_flat = view_matrix.flatten()
        f_in_view.write("".join([to_hex_16(Q_MAIN, x) for x in view_flat]) + "\n")
        
        processed_gaussians = []
        
        for g_data in gaussians_data:
            # Write Input
            pos_in = Q_MAIN.quantize(g_data['pos'])
            scale_in = Q_MAIN.quantize(g_data['scale'])
            rot_in = Q_MAIN.quantize(g_data['rot'])
            opacity_in = Q_MAIN.quantize(g_data['opacity'])
            color_in = Q_COLOR.quantize(g_data['color'])
            color_degraded_val = np.floor(color_in * 64 + 0.5) / 64.0
            
            inputs = np.concatenate([pos_in, scale_in, rot_in, [opacity_in], color_degraded_val])
            f_in_gauss.write("".join([to_hex_16(Q_MAIN, x) for x in inputs]) + "\n")
            
            # Internal
            pos = Q_MAIN.quantize(g_data['pos'])
            scale = Q_MAIN.quantize(g_data['scale'])
            rot = Q_MAIN.quantize(g_data['rot'])
            opacity = Q_MAIN.quantize(g_data['opacity'])
            color = color_degraded_val
            
            # Stage 1
            rot_mat = quat_to_rotmat(rot)
            f_rot.write("".join([to_hex_16(Q_MAIN, x) for x in rot_mat.flatten()]) + "\n")
            
            # Stage 2
            scale_mat = compute_scale_mat(scale)
            cov3d = compute_cov3d(scale_mat, rot_mat)
            f_cov3d.write("".join([to_hex_16(Q_MAIN, x) for x in cov3d.flatten()]) + "\n")
            
            # Stage 3
            cam_pos, cam_pos_int = compute_cam_pos_hw(pos, view_matrix)
            f_cam.write("".join([to_hex_16(Q_MAIN, x) for x in cam_pos]) + "\n")
            
            z_int = cam_pos_int[2]
            z = cam_pos[2]
            
            # Hardware check: campos_z < 13 (approx 0.2 * 64)
            if z_int < 13:
                 f_jac.write("0" * 4 * 6 + "\n")
                 f_cov2d.write("0" * 4 * 4 + "\n")
                 f_conic.write("0" * 4 * 3 + "\n")
                 f_param.write("0" * 32 + "\n")
                 # Invalid Gaussian. 
                 # In HW, it gets Depth 0xFFFF. We use a large float for sorting, 
                 # but for writing file we will use 0xFFFF manually.
                 processed_gaussians.append({
                    'valid': False, 
                    'id': g_data['id'], 
                    'depth': 9999.0, # Large depth to sort to end
                    'depth_hw': 0xFFFF
                 })
                 continue
                 
            # Stage 4
            J = compute_jacobian(cam_pos, proj_matrix, width, height)
            f_jac.write("".join([to_hex_16(Q_MAIN, x) for x in J.flatten()]) + "\n")
            
            # Stage 5 & 6 & BBox
            j00 = J[0,0]
            j11 = J[1,1]
            def quantize_floor(val): return math.floor(val * 64) / 64
            screen_x = cam_pos[0] * j00 + width/2
            screen_y = cam_pos[1] * j11 + height/2
            screen_pos = np.array([quantize_floor(screen_x), quantize_floor(screen_y)])
            
            cov2d, cov2d_unclamped = compute_cov2d(cov3d, view_matrix, J)
            min_x, max_x, min_y, max_y, rad_x, rad_y = compute_bbox(screen_pos, cov2d_unclamped, width, height)
            bbox = (min_x, max_x, min_y, max_y, rad_x, rad_y)
            conic = compute_conic(cov2d)
            
            f_cov2d.write("".join([to_hex_16(Q_MAIN, x) for x in cov2d.flatten()]) + "\n")
            f_conic.write("".join([to_hex_16(Q_CONIC, x) for x in conic]) + "\n")
            
            sx_h = to_hex_16(Q_MAIN, screen_pos[0])
            sy_h = to_hex_16(Q_MAIN, screen_pos[1])
            minx_h = f"{bbox[0]:04x}"
            maxx_h = f"{bbox[1]:04x}"
            miny_h = f"{bbox[2]:04x}"
            maxy_h = f"{bbox[3]:04x}"
            f_bbox.write(f"{sx_h}{sy_h}{minx_h}{maxx_h}{miny_h}{maxy_h}\n")
            
            rad_x_val = bbox[4]
            rad_y_val = bbox[5]
            ca_h = to_hex_16(Q_CONIC, conic[0])
            cb_h = to_hex_16(Q_CONIC, conic[1])
            cc_h = to_hex_16(Q_CONIC, conic[2])
            cx_h = to_hex_16(Q_MAIN, screen_pos[0])
            cy_h = to_hex_16(Q_MAIN, screen_pos[1])
            rx_h = to_hex_16(Q_MAIN, rad_x_val)
            ry_h = to_hex_16(Q_MAIN, rad_y_val)
            res_h = "0000"
            f_param.write(f"{res_h}{ry_h}{rx_h}{cy_h}{cx_h}{cc_h}{cb_h}{ca_h}\n")
            
            # For valid gaussians, depth_hw is the quantized Q10.6 value in hex integer
            depth_quant = int(math.floor(z * 64 + 0.5)) & 0xFFFF
            
            processed_gaussians.append({
                'valid': True, 
                'id': g_data['id'], 
                'depth': z,
                'depth_hw': depth_quant,
                'screen_pos': screen_pos, 'conic': conic, 'opacity': opacity, 'color': color, 'bbox': bbox
            })
            
        # Stage 7 - Sort
        # Sort key: Depth Ascending, then ID Ascending (for stable sort when depth equal)
        # CRITICAL: Use depth_hw (Quantized) to match Hardware Sort behavior EXACTLY.
        # Hardware sees uniform quantized values so it encounters "equal depth" more often than float sort.
        processed_gaussians.sort(key=lambda x: (x['depth_hw'], x['id']))
        
        for g in processed_gaussians:
            # HW Format: {PADDING(9), ID(7), DEPTH(16)} -> 32 bits
            # ID is in bits [22:16]. Depth in [15:0].
            # g['id'] is integer.
            
            hw_id = g['id'] & 0x7F
            hw_depth = g['depth_hw'] & 0xFFFF
            pack_val = (hw_id << 16) | hw_depth
            
            f_sort.write(f"{pack_val:08x}\n")
            
        # Stage 8 - Render
        valid_gaussians = [g for g in processed_gaussians if g['valid']]
        
        canvas, sram_int = simulate_accumulation_hardware_flow(valid_gaussians, width, height, s)
        
        for y in range(height):
            for x in range(width):
                t_val = sram_int[y, x, 0]
                r_val = sram_int[y, x, 1]
                g_val = sram_int[y, x, 2]
                b_val = sram_int[y, x, 3]
                t_h = f"{t_val:04x}"
                r_h = f"{r_val & 0xFFFF:04x}"
                g_h = f"{g_val & 0xFFFF:04x}"
                b_h = f"{b_val & 0xFFFF:04x}"
                f_img.write(f"{t_h}{r_h}{g_h}{b_h}\n")
                
        # Save Visual
        img_uint8 = np.clip(canvas * 255, 0, 255).astype(np.uint8)
        Image.fromarray(img_uint8).save(os.path.join(DIRS["visual"], f"render_{s:02d}.png"))
        
        if scene_type == 'ICLAB':
            frames_iclab.append(img_uint8)
        else:
            frames_cornell.append(img_uint8)
            
        f_in_gauss.close()
        f_in_view.close()
        f_rot.close()
        f_cov3d.close()
        f_cam.close()
        f_jac.close()
        f_cov2d.close()
        f_conic.close()
        f_bbox.close()
        f_sort.close()
        f_param.close()
        f_img.close()

    # Save Videos
    print("Saving Videos...")
    if frames_iclab:
        imageio.mimsave(os.path.join(OUTPUT_DIR, "demo_iclab.mp4"), frames_iclab, fps=30)
        imageio.mimsave(os.path.join(OUTPUT_DIR, "demo_iclab.gif"), frames_iclab, fps=10, loop=0)
    if frames_cornell:
        # Create ping-pong loop (forward + reverse)
        frames_cornell_extended = frames_cornell + frames_cornell[::-1]
        imageio.mimsave(os.path.join(OUTPUT_DIR, "demo_cornell.mp4"), frames_cornell_extended, fps=30)
        imageio.mimsave(os.path.join(OUTPUT_DIR, "demo_cornell.gif"), frames_cornell_extended, fps=10, loop=0)
        
    print("Done.")

if __name__ == "__main__":
    generate_data()
