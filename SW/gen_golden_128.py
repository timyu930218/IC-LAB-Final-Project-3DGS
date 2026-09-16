import numpy as np
import os
import math
import struct
from PIL import Image

# ==========================================
# Configuration
# ==========================================
NUM_PATTERNS = 153
OUTPUT_DIR = "golden_128"

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
        # floor(x + 0.5)
        val = math.floor(x * self.scale + 0.5)
        
        min_int = -(1 << (self.total_bits - 1))
        max_int = (1 << (self.total_bits - 1)) - 1
        
        if val > max_int: val = max_int
        if val < min_int: val = min_int
        
        return val / self.scale

    def quantize_trunc(self, x):
        # Round Towards Zero (Truncation) for Division
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
Q_DET_INV = Quantizer(14, 18) # 32-bit intermediate (changed from 48-bit sim)
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
    # q = [w, x, y, z]
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
    # M = R @ S
    M = rot_mat @ scale_mat
    M = Q_MAIN.quantize(M)
    Sigma = M @ M.T
    return Q_MAIN.quantize(Sigma)

def compute_cam_pos(xyz, view_matrix):
    xyz_hom = np.append(xyz, 1.0)
    t = view_matrix @ xyz_hom
    return Q_MAIN.quantize(t[:3])

def compute_jacobian(t, proj_matrix, width, height):
    x, y, z = t
    
    # Focal lengths
    fx = proj_matrix[0,0] * width / 2
    fy = proj_matrix[1,1] * height / 2
    
    # Jacobian (d_screen / d_cam)
    # Y-Down system
    # Use Truncation for division results to match hardware
    
    j00 = Q_MAIN.quantize_trunc(fx/z)
    j02 = Q_MAIN.quantize_trunc(-(fx*x)/(z**2))
    j11 = Q_MAIN.quantize_trunc(-fy/z)
    j12 = Q_MAIN.quantize_trunc((fy*y)/(z**2))
    
    J = np.array([
        [j00, 0, j02],
        [0, j11, j12]
    ])
    # J elements are already quantized. 
    # Returning J directly.
    return J

def compute_cov2d(cov3d, view_matrix, jacobian):
    W = view_matrix[:3, :3]
    Sigma_cam = W @ cov3d @ W.T
    Sigma_cam = Q_MAIN.quantize(Sigma_cam)
    
    cov2d = jacobian @ Sigma_cam @ jacobian.T
    cov2d = Q_MAIN.quantize(cov2d)
    
    # Low pass
    # Hardware adds 19 (Q10.6). 19/64.0 = 0.296875
    # Using 0.3 results in mismatches when rounding logic is sensitive
    offset = 19.0 / 64.0
    cov2d[0,0] += offset
    cov2d[1,1] += offset
    
    # 2024-12-16 Hardware Mismatch Fix
    # Hardware Stage 5 calculates BBox using the RAW (Unclamped) accumulator value.
    # However, it outputs the CLAMPED (Saturated) value to Stage 6 (Conic) and SRAM.
    # We must replicate this split behavior.
    
    cov2d_clamped = Q_MAIN.quantize(cov2d)
    
    # For BBox, we need the "Raw" integer value that represents the 40-bit accumulator result.
    # The accumulator is effectively: cov2d_unclamped_q10_6
    # effectively just the float value quantized but NOT clamped.
    # We simulate this by just taking the quantized float (without range check)
    
    def quantize_no_clamp(arr, fract_bits=6):
        scale = (1 << fract_bits)
        return np.floor(arr * scale + 0.5) / scale

    cov2d_unclamped = quantize_no_clamp(cov2d)
    
    return cov2d_clamped, cov2d_unclamped

def compute_conic(cov2d):
    # Hardware-Accurate Integer Implementation matching Stage6_Conic.v
    
    # 1. Inputs to Integer Q10.6
    # cov2d elements are already quantized floats (Q10.6)
    sigma00 = to_signed(to_fixed(cov2d[0,0], 64), 16)
    sigma01 = to_signed(to_fixed(cov2d[0,1], 64), 16)
    sigma11 = to_signed(to_fixed(cov2d[1,1], 64), 16)
    
    # 2. Det Calculation (Q20.12)
    # mult1 = sigma00 * sigma11
    # mult2 = sigma01 * sigma01 (sigma01 is sigma10 for symmetric)
    mult1 = to_signed(sigma00 * sigma11, 32)
    mult2 = to_signed(sigma01 * sigma01, 32)
    
    det_q = to_signed(mult1 - mult2, 32)
    
    # 3. Det Inv Calculation (Q1.23 target, effectively)
    # logic: det_inv = (2^35 + det_abs/2) / det_abs
    # Sign handled separately
    
    if det_q < 0:
        sign = -1
        det_abs = -det_q
    else:
        sign = 1
        det_abs = det_q
        
    if det_abs == 0: det_abs = 1
    
    # integer division with rounding
    # term = (1<<35) + (det_abs >> 1)
    numerator = (1 << 35) + (det_abs >> 1)
    det_inv_abs = numerator // det_abs
    
    if sign == -1:
        det_inv_q = -det_inv_abs
    else:
        det_inv_q = det_inv_abs
        
    # det_inv_q is approx Q1.23 (or scaled by 2^35/2^12 = 2^23)
    # Hardware var is 36-bit.
    
    # 4. Conic Coefficients (Q4.12)
    # conic_a = (sigma11 * det_inv + (1<<16)) >>> 17
    # conic_b = (-sigma01 * det_inv + (1<<16)) >>> 17
    # conic_c = (sigma00 * det_inv + (1<<16)) >>> 17
    
    # Products are roughly 16-bit * 36-bit -> 52-bit.
    
    term_a = sigma11 * det_inv_q
    conic_a_int = (term_a + (1 << 16)) >> 17
    
    term_b = (-sigma01) * det_inv_q
    conic_b_int = (term_b + (1 << 16)) >> 17
    
    term_c = sigma00 * det_inv_q
    conic_c_int = (term_c + (1 << 16)) >> 17
    
    # Wrap to 16-bit
    conic_a_int = to_signed(conic_a_int, 16)
    conic_b_int = to_signed(conic_b_int, 16)
    conic_c_int = to_signed(conic_c_int, 16)
    
    # Convert back to float for output compatibility (Q4.12 -> float)
    return np.array([conic_a_int, conic_b_int, conic_c_int]) / 4096.0

def compute_bbox(center, cov2d, width, height):
    if width == 128:
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
        # (Raw + 2048) >> 12 is handled by our int() conversion of Q10.6 value (it's the integer rep)
        # Wait, cov00_int IS the Q10.6 integer representation.
        # So it IS roughly (Raw_Full >> 12).
        # But we need (Raw_Full >> 12) >> 2. which is raw_x >> 2.
        
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
        
    else:
        # 64 Mode (Original Logic)
        rad_x = 3.0 * math.sqrt(max(0.05, cov2d[0,0]))
        rad_y = 3.0 * math.sqrt(max(0.05, cov2d[1,1]))
        
        rad_x = math.floor(rad_x * 64 + 0.5) / 64.0
        rad_y = math.floor(rad_y * 64 + 0.5) / 64.0
    
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

def load_exp_lut():
    global EXP_LUT
    if EXP_LUT is not None:
        return

    lut_path = os.path.join(os.path.dirname(__file__), "../hdl/exp_lut.v")
    EXP_LUT = [0] * 2048
    
    try:
        with open(lut_path, "r") as f:
            for line in f:
                line = line.strip()
                # Parse lines like: 11'd0: data = 16'h0100;
                if "data =" in line and "11'd" in line:
                    parts = line.split(":")
                    if len(parts) >= 2:
                        addr_str = parts[0].split("'d")[1].strip()
                        data_part = parts[1].split("=")[1].strip()
                        # data_part might look like "16'h0100;" or "16'h0100"
                        hex_str = data_part.split("'h")[1].split(";")[0].strip()
                        
                        addr = int(addr_str)
                        val = int(hex_str, 16)
                        
                        if 0 <= addr < 2048:
                            EXP_LUT[addr] = val
        print("Loaded exp_lut.v successfully.")
    except Exception as e:
        print(f"Error loading exp_lut.v: {e}")
        # Fallback to approximation if file missing (should not happen in dev)
        raise e

def hardware_exp(power_int):
    # Input: power_int is Q4.12 integer (negative magnitude usually)
    # Hardware P is signed. 
    
    # Ensure LUT is loaded
    load_exp_lut()
    
    p_val = power_int
    if p_val > 32767: p_val -= 65536 # Convert unsigned 16-bit view to signed python int
    
    if p_val > 0: return 0.0 # Should not happen if caller checks
    
    p_abs = abs(p_val)
    
    # Address extraction: bits [14:4] of Q4.12 absolute value
    lut_addr_full = p_abs >> 4 # extract higher bits
    
    if lut_addr_full > 2047: 
        lut_addr = 2047
    else:
        lut_addr = lut_addr_full & 0x7FF
    
    # Fetch Q8.8 value
    val_q88 = EXP_LUT[lut_addr]
    
    # Return as float for compatibility with rest of pipeline
    return val_q88 / 256.0

# Helper for signed wrapping
def to_signed(val, bits):
    mask = (1 << bits) - 1
    val_masked = val & mask
    if val_masked & (1 << (bits - 1)):
        return val_masked - (1 << bits)
    return val_masked

def to_fixed(val, scale):
    # Matches to_hex_16 logic: floor(val * scale + 0.5)
    return int(math.floor(val * scale + 0.5))

def evaluate_gaussian(s, pixel_x, pixel_y, center, conic, opacity):
    # Precise simulation of Stage8_Render.v integer arithmetic
    
    # 1. Inputs to Integer Domain
    # pixel coordinates are integers (0..63)
    # center is Q10.6. Convert to integer representation.
    center_x_int = to_signed(to_fixed(center[0], 64), 16)
    center_y_int = to_signed(to_fixed(center[1], 64), 16)
    
    # conic is Q4.12
    conic_a_int = to_signed(to_fixed(conic[0], 4096), 16)
    conic_b_int = to_signed(to_fixed(conic[1], 4096), 16)
    conic_c_int = to_signed(to_fixed(conic[2], 4096), 16)
    
    # 2. Delta Calculation (Q10.6)
    # assign delta_x = {2'b0, x, 6'b0} - center_x;
    # (pixel_x << 6) is equivalent to {x, 6'b0}
    # x is 8-bit unsigned effectively.
    x_shifted = pixel_x << 6
    y_shifted = pixel_y << 6
    
    # delta_x is 16-bit signed
    delta_x_int = to_signed(x_shifted - center_x_int, 16)
    delta_y_int = to_signed(y_shifted - center_y_int, 16)
    
    # 3. Power Calculation
    # wire signed [31:0] dx_sq = delta_x * delta_x; // Q20.12
    # Products are 32-bit signed
    dx_sq = to_signed(delta_x_int * delta_x_int, 32)
    dy_sq = to_signed(delta_y_int * delta_y_int, 32)
    dx_dy = to_signed(delta_x_int * delta_y_int, 32)
    
    # Intermediate terms (Q24.24) -> 48 bits
    # wire signed [47:0] term_a = conic_a * dx_sq; 
    # conic_a (16) * dx_sq (32) = 48 bits
    term_a = to_signed(conic_a_int * dx_sq, 48)
    term_b = to_signed(conic_b_int * dx_dy, 48)
    term_c = to_signed(conic_c_int * dy_sq, 48)
    
    # wire signed [47:0] sum_terms = term_a + (term_b <<< 1) + term_c;
    # term_b shifted left by 1. Check wrap on shift?
    # (term_b * 2) must fit in 48 bits? Or does it overflow?
    # Verilog (term_b <<< 1) is 48-bit expression?
    # IF term_b is 48 bit. <<< 1 shifts out MSB?
    # Yes. (term_b <<< 1) is 48-bit result.
    term_b_shifted = to_signed(term_b << 1, 48)
    
    # Sum
    sum_terms = to_signed(term_a + term_b_shifted + term_c, 48)
    
    # Negate and Shift Right by 1 (Divide by 2)
    # wire signed [47:0] power_temp = -(sum_terms >>> 1);
    # sum_terms >>> 1. Arithmetic shift.
    sum_shifted = sum_terms >> 1 
    # Negate. 48-bit wrap?
    power_temp = to_signed(-sum_shifted, 48)
    
    # Clamp and Normalize to Q4.12
    # wire signed [47:0] power_shifted = (power_temp + (1<<11)) >>> 12;
    # Rounding added: + 2048 (which is 1<<11) before shifting 12
    power_shifted = to_signed(power_temp + 2048, 48) >> 12
    
    # assign P = (power_shifted < -48'sd32768) ? 16'h8000 : power_shifted[15:0];
    MIN_VAL = -32768
    if power_shifted < MIN_VAL:
        P = MIN_VAL # Reprsented as 16'h8000
    else:
        # P takes lower 16 bits.
        # Check if it fits in signed 16-bit?
        # Only check max positive.
        if power_shifted > 32767:
            P = 32767 # Clamp positive?
        else:
            # If my Python code sets P = 40000? not valid for 16-bit var used later.
            # I should simulate P as 16-bit signed wire.
            P = to_signed(power_shifted, 16)
            
    # 4. Check Power Condition
    # if (P > 0) alpha = 0;
    if P > 0:
        return 0.0
        
    # 5. EXP LUT Lookup
    # hardware_exp now takes the integer P
    val_float = hardware_exp(P) # Returns float 0.0-1.0
    val_q88_int = int(val_float * 256 + 0.5) # Re-quantize to Q8.8 just to be sure we have integer
    
    # 6. Alpha Calculation
    # alpha = opacity * val
    # opacity is Q10.6
    # val is Q8.8 (exp_lut_out)
    
    opacity_int = to_signed(to_fixed(opacity, 64), 16) # Q10.6
    
    # wire signed [31:0] alpha_prod = {{16{color_t[15]}}, color_t} * {16'b0, exp_lut_out};
    # Product is Q10.6 * Q8.8 = Q18.14
    # 32-bit signed product
    alpha_prod = to_signed(opacity_int * val_q88_int, 32)
    
    # Rounding to Q8.8
    # wire signed [31:0] alpha_rounded = (alpha_prod + 32) >>> 6;
    alpha_rounded = (alpha_prod + 32) >> 6
    # Keep alpha_rounded as 32-bit signed (though it should be small positive)
    alpha_rounded = to_signed(alpha_rounded, 32)
    
    # Clamping Logic matching hardware (Stage8_Render.v)
    # else if (alpha_rounded <= 32'd1) alpha = 16'd0;
    # else if (alpha_rounded > 32'sd253) alpha = 16'd253;
    # else alpha = alpha_rounded[15:0];
    
    if alpha_rounded <= 1:
        alpha_final_int = 0
    elif alpha_rounded > 253:
        alpha_final_int = 253
    else:
        alpha_final_int = alpha_rounded
        
    # if (s > 100):
        # print(f"DEBUG EVALUATE SCENE {s} PIXEL ({pixel_x}, {pixel_y}):")
        # print(f"  center: {center_x_int}, {center_y_int}")
        # print(f"  conic: {conic_a_int}, {conic_b_int}, {conic_c_int}")
        # print(f"  delta: {delta_x_int}, {delta_y_int}")
        # print(f"  dx_sq: {dx_sq}, dy_sq: {dy_sq}, dx_dy: {dx_dy}")
        # print(f"  term_a: {term_a}, term_b: {term_b}, term_c: {term_c}")
        # print(f"  sum_terms: {sum_terms}")
        # print(f"  power_temp: {power_temp}")
        # print(f"  power_shifted: {power_shifted}")
        # print(f"  P: {P} (P_abs: {abs(P)})")
        # print(f"  lut_addr: {lut_addr}")
        # print(f"  val_q88: {val_q88_int}")
        # print(f"  opacity: {opacity_int}")
        # print(f"  alpha_prod: {alpha_prod}")
        # print(f"  alpha_rounded: {alpha_rounded}")
        # print(f"  alpha_final: {alpha_final_int}")

    return alpha_final_int / 256.0

def simulate_accumulation_hardware_flow(gaussians_sorted, width, height, s_idx):
    # =========================================================
    # Hardware Accumulation Logic (Rasterization / Read-Modify-Write)
    # =========================================================
    
    # SRAM Buffer Simulation
    # Format: [T(Q0.8), R(Q10.6), G(Q10.6), B(Q10.6)]
    # Use int32 to hold values, avoiding overflow before clamp (though hardware handles wrap/clamp)
    # Initialize: T=255 (1.0), RGB=0
    sram = np.zeros((height, width, 4), dtype=np.int32)
    sram[:, :, 0] = 256 
    
    for g in gaussians_sorted:
        # Iterate Bounding Box
        min_x, max_x, min_y, max_y = g['bbox'][:4]
        
        # Hardware iterates over the generic screen space but checks bbox bounds
        # Here we only iterate the bbox for speed, which is logically equivalent 
        # as long as we don't write outside.
        
        for y in range(min_y, max_y + 1):
            if y < 0 or y >= height: continue
            for x in range(min_x, max_x + 1):
                if x < 0 or x >= width: continue
                
                # 1. Calculate Alpha
                alpha_float = evaluate_gaussian(s_idx, x, y, g['screen_pos'], g['conic'], g['opacity'])
                
                # Convert to Q0.8 (0..255)
                # evaluate_gaussian returns (int_val / 256.0), so we reverse that.
                alpha_q8 = int(alpha_float * 256 + 0.5)
                
                # Early exit if alpha is 0 (optimization, hardware might still pipeline it)
                if alpha_q8 <= 0: continue
                
                # 2. Read from SRAM
                t_old = sram[y, x, 0]
                r_old = sram[y, x, 1]
                g_old = sram[y, x, 2]
                b_old = sram[y, x, 3]
                
                # Stop if transmittance is saturated (close to 0)
                # Hardware might skip if T < threshold, or just compute anyway.
                if t_old == 0: continue
                
                # 3. Calculate Weight
                # w = alpha * T
                # Product Q0.8 * Q0.8 = Q0.16 -> Shift 8 -> Q0.8
                weight_q8 = (alpha_q8 * t_old) >> 8
                
                # 4. Color Contribution
                # contrib = color * weight
                # color is Q10.6
                # We need the Integer representation of the gaussian color (Q10.6)
                # g['color'] is float 0.0-1.0 (degraded).
                
                c_r_int = int(g['color'][0] * 64 + 0.5)
                c_g_int = int(g['color'][1] * 64 + 0.5)
                c_b_int = int(g['color'][2] * 64 + 0.5)
                
                if s_idx == 106 and x == 23 and y == 63:
                     print(f"DEBUG ACCUM SCENE 106 PIXEL (23, 63) ID {g['id']}:")
                     print(f"  alpha_q8: {alpha_q8}")
                     print(f"  weight_q8: {weight_q8}")
                     # print(f"  c_r_int: {c_r_int} (float: {g['color'][0]})")
                     # print(f"  c_b_int: {c_b_int} (float: {g['color'][2]})")
                     # print(f"  prod_r: {c_r_int * weight_q8}")
                     # print(f"  sum_r: {c_r_int * weight_q8 + (r_old << 8)}")
                
                # 5. Accumulate Color (Read-Add-Write) - Hardware Match
                # Hardware accumulates in Q10.14 (Color[Q10.6]*Weight[Q0.8])
                # Formula: Final = Round_Q14_to_Q6( (Old_Color << 8) + (Color * Weight) )
                
                # Product: Q10.6 * Q0.8 = Q10.14
                prod_r = c_r_int * weight_q8 
                prod_g = c_g_int * weight_q8 
                prod_b = c_b_int * weight_q8 
                
                # Accumulate in Q10.14
                sum_r = prod_r + (r_old << 8)
                sum_g = prod_g + (g_old << 8)
                sum_b = prod_b + (b_old << 8)
                
                # Hardware 2-Stage Rounding
                # Stage 1: Q10.14 -> Q10.8 (Offset 32, Shift 6)
                mid_r = (sum_r + 32) >> 6
                mid_g = (sum_g + 32) >> 6
                mid_b = (sum_b + 32) >> 6
                
                # Stage 2: Q10.8 -> Q10.6 (Offset 2, Shift 2)
                r_new = (mid_r + 2) >> 2
                g_new = (mid_g + 2) >> 2
                b_new = (mid_b + 2) >> 2
                
                # Clamp (if needed, though Q10.6 allows growth)
                # Hardware uses 16-bit wires usually, but let's assume valid range.

                
                # 6. Update Transmittance
                # T_new = T_old * (1 - alpha)
                # (1 - alpha) in Q0.8 is (256 - alpha_q8)
                inv_alpha = 256 - alpha_q8
                
                # Product Q0.8 * Q0.8 -> Q0.16 -> Shift 8 -> Q0.8
                t_new = (t_old * inv_alpha) >> 8
                
    # 7. Write Back
                sram[y, x, 0] = t_new
                sram[y, x, 1] = r_new
                sram[y, x, 2] = g_new
                sram[y, x, 3] = b_new
                
                if s_idx == 0 and x == 29 and y == 42:
                    pass
                    # Debug trace for specific pixel
                     # print(f"DEBUG ACCUM Pixel (29,42) | G_ID: {g['id']}")
                     # print(f"  alpha_q8: {alpha_q8}")
                     # print(f"  T_old: {t_old} -> T_new: {t_new}")
                     # print(f"  weight_q8: {weight_q8}")
                     # print(f"  ColorG(R): {c_r_int}")
                     # print(f"  Prod(R): {prod_r}")
                     # print(f"  Sum(R): {sum_r} -> Mid: {mid_r}")
                     # print(f"  R_old: {r_old} -> R_new: {r_new}")

    # Return Canvas in Float 0..1 format for image saving
    # Color Q10.6 -> Float
    final_canvas = np.zeros((height, width, 3))
    final_canvas[:, :, 0] = sram[:, :, 1] / 64.0
    final_canvas[:, :, 1] = sram[:, :, 2] / 64.0
    final_canvas[:, :, 2] = sram[:, :, 3] / 64.0
    
    return final_canvas, sram


# ==========================================
# Scene Content Generators (New 128x128 Scenes)
# ==========================================

def create_cube_scene():
    gaussians = []
    # 3D Grid 4x4x4 = 64 Gaussians
    steps = np.linspace(-1.5, 1.5, 4)
    i = 0
    for x in steps:
        for y in steps:
            for z in steps:
                if i >= 64: break
                pos = np.array([x, y, z + 2.0]) # Shift z to be visible
                scale = np.array([0.2, 0.2, 0.2])
                rot = np.array([1.0, 0.0, 0.0, 0.0])
                # Gradient Color based on position
                r = (x + 1.5) / 3.0
                g = (y + 1.5) / 3.0
                b = (z + 1.5) / 3.0
                color = np.array([r, g, b])
                opacity = 0.9
                gaussians.append({
                    'pos': pos, 'scale': scale, 'rot': rot, 'opacity': opacity, 'color': color, 'id': i
                })
                i += 1
    return gaussians

def create_helix_scene():
    gaussians = []
    num_points = 64
    for i in range(num_points):
        t = i / (num_points - 1)
        angle = 4 * math.pi * t # 2 turns
        radius = 1.0
        x = radius * math.cos(angle)
        y = -2.0 + 4.0 * t
        z = radius * math.sin(angle) + 2.0
        
        pos = np.array([x, y, z])
        scale = np.array([0.15, 0.15, 0.15])
        
        # Orient along the helix tangent? Simplify to identity for now.
        rot = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Rainbow color
        hue = t
        # Simple Hue to RGB approx
        def hue_to_rgb(h):
            r = abs(h * 6 - 3) - 1
            g = 2 - abs(h * 6 - 2)
            b = 2 - abs(h * 6 - 4)
            return np.clip([r, g, b], 0, 1)
            
        color = hue_to_rgb(hue)
        opacity = 0.9
        gaussians.append({
            'pos': pos, 'scale': scale, 'rot': rot, 'opacity': opacity, 'color': color, 'id': i
        })
    return gaussians

def create_sphere_scene():
    gaussians = []
    num_points = 64
    golden_ratio = (1 + 5**0.5) / 2
    for i in range(num_points):
        t = i / num_points
        inclination = math.acos(1 - 2 * (i + 0.5) / num_points)
        azimuth = 2 * math.pi * golden_ratio * i
        
        radius = 1.5
        x = radius * math.sin(inclination) * math.cos(azimuth)
        y = radius * math.sin(inclination) * math.sin(azimuth)
        z = radius * math.cos(inclination) + 2.0
        
        pos = np.array([x, y, z])
        scale = np.array([0.15, 0.15, 0.15])
        rot = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Color based on normal (x,y,z normalized is direction from center)
        # Map -1..1 to 0..1
        color = (np.array([x, y, z - 2.0]) / radius + 1.0) / 2.0
        opacity = 0.9
        
        gaussians.append({
            'pos': pos, 'scale': scale, 'rot': rot, 'opacity': opacity, 'color': color, 'id': i
        })
    return gaussians

# ==========================================
# Main Generation Loop
# ==========================================

def generate_data():
    print(f"Generating {NUM_PATTERNS} scenes...")
    
    width, height = 128, 128
    
    # Projection Matrix
    # Fixed Focal Lengths to match Hardware constants for 128 mode
    # Hardware fx = 7104 (Q10.6) => 111.0
    fx_target = 111.0
    fy_target = 111.0
    
    # Calculate projection matrix elements (P00, P11)
    # such that P00 * (width/2) = fx_target
    p00 = fx_target / (width / 2.0)
    p11 = fy_target / (height / 2.0)
    
    proj_matrix = np.array([
        [p00, 0, 0, 0],
        [0, p11, 0, 0],
        [0, 0, 1, -0.2],
        [0, 0, 1, 0]
    ])
    proj_matrix = Q_MAIN.quantize(proj_matrix)
    
    for s in range(NUM_PATTERNS):
        if s == 106: print(f"DEBUG: Processing SCENE {s}...")
        # File handles
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
        print(f"DEBUG, Generating {s} test case")
        # Scene Config
        # Scene Config
        # 0-49: 1 Gaussian
        # 50-99: 4 Gaussians
        # 100-149: 64 Gaussians (Random)
        # 150-152: Structured Scenes (Cube, Helix, Sphere)

        gaussians_data = []

        if s == 150:
             gaussians_data = create_cube_scene()
             num_gaussians = len(gaussians_data)
        elif s == 151:
             gaussians_data = create_helix_scene()
             num_gaussians = len(gaussians_data)
        elif s == 152:
             gaussians_data = create_sphere_scene()
             num_gaussians = len(gaussians_data)
        else:
            if s < 50:
                num_gaussians = 1
            elif s < 100:
                num_gaussians = 4
            else:
                num_gaussians = 64
                
            for i in range(num_gaussians):
                pos = np.array([np.random.rand()*2-1, np.random.rand()*2-1, np.random.rand()*2+1])
                scale = np.array([np.random.rand()*0.5+0.2, np.random.rand()*0.5+0.2, np.random.rand()*0.5+0.2])
                # Random Rotation (Quaternion)
                u1, u2, u3 = np.random.rand(3)
                sqrt_1_minus_u1 = math.sqrt(1 - u1)
                sqrt_u1 = math.sqrt(u1)
                theta1 = 2 * math.pi * u2
                theta2 = 2 * math.pi * u3
                
                w = sqrt_1_minus_u1 * math.sin(theta1)
                x = sqrt_1_minus_u1 * math.cos(theta1)
                y = sqrt_u1 * math.sin(theta2)
                z = sqrt_u1 * math.cos(theta2)
                
                rot = np.array([w, x, y, z])
                color = np.random.rand(3)
                opacity = 0.8 + np.random.rand() * 0.2
                
                gaussians_data.append({
                    'pos': pos, 'scale': scale, 'rot': rot, 'opacity': opacity, 'color': color, 'id': i
                })
            
        # Camera
        eye = np.array([0.0, 0.0, 5.0])
        target = np.array([0.0, 0.0, 2.0])
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
        
        # Write View
        view_flat = view_matrix.flatten()
        f_in_view.write("".join([to_hex_16(Q_MAIN, x) for x in view_flat]) + "\n")
        
        processed_gaussians = []
        
        for g_data in gaussians_data:
            # Write Input (Q_MAIN - 16-bit Q10.6)
            # We use Q_MAIN for all input fields for uniformity in 16-bit
            pos_in = Q_MAIN.quantize(g_data['pos'])
            scale_in = Q_MAIN.quantize(g_data['scale'])
            rot_in = Q_MAIN.quantize(g_data['rot'])
            opacity_in = Q_MAIN.quantize(g_data['opacity'])
            color_in = Q_COLOR.quantize(g_data['color']) # Use Q_COLOR (Q8.8) to match internal sim
            
            # Degrade color to Q10.6 to match hardware input (which reads from file stored as Q10.6)
            # This ensures software and hardware use the exact same color values
            color_degraded_val = np.floor(color_in * 64 + 0.5) / 64.0
            
            inputs = np.concatenate([pos_in, scale_in, rot_in, [opacity_in], color_degraded_val])
            f_in_gauss.write("".join([to_hex_16(Q_MAIN, x) for x in inputs]) + "\n")
            
            # Internal (Q_MAIN)
            pos = Q_MAIN.quantize(g_data['pos'])
            scale = Q_MAIN.quantize(g_data['scale'])
            rot = Q_MAIN.quantize(g_data['rot'])
            opacity = Q_MAIN.quantize(g_data['opacity'])
            color = color_degraded_val
            
            # Stage 1: Rot Mat
            rot_mat = quat_to_rotmat(rot)
            f_rot.write("".join([to_hex_16(Q_MAIN, x) for x in rot_mat.flatten()]) + "\n")
            
            # Stage 2: Cov3D
            scale_mat = compute_scale_mat(scale)
            cov3d = compute_cov3d(scale_mat, rot_mat)
            f_cov3d.write("".join([to_hex_16(Q_MAIN, x) for x in cov3d.flatten()]) + "\n")
            
            # Stage 3: Cam Pos
            cam_pos = compute_cam_pos(pos, view_matrix)
            f_cam.write("".join([to_hex_16(Q_MAIN, x) for x in cam_pos]) + "\n")
            
            # Check Cull
            z = cam_pos[2]
            if z <= 0.2:
                # Invalid - Write 0s for rest
                f_jac.write("0" * 4 * 6 + "\n")
                f_cov2d.write("0" * 4 * 4 + "\n") # 2x2
                f_conic.write("0" * 4 * 3 + "\n")
                f_param.write("0" * 32 + "\n")
                processed_gaussians.append({'valid': False, 'id': g_data['id'], 'depth': 0})
                continue
                
            # Stage 4: Jacobian
            J = compute_jacobian(cam_pos, proj_matrix, width, height)
            f_jac.write("".join([to_hex_16(Q_MAIN, x) for x in J.flatten()]) + "\n")
            
            # Stage 5: Cov2D
            # cov3d already computed in Stage 2
            # cov2d computation moved to later block to support BBox calculation logic
            # cov2d = compute_cov2d(cov3d, view_matrix, J)
            # f_cov2d.write(...)
            
            # Defer writing Cov2D until it is computed correctly
            # defer Conic too
            
            # Stage 6: Conic
            # conic = compute_conic(cov2d)
            # f_conic.write(...)
            
            # Screen Pos for Raster
            # Use Jacobian elements to match hardware (j00 = fx/z, j11 = -fy/z)
            # Hardware: u = (x * j00 + width/2) >>> 6 (Floor)
            # Hardware: v = (y * j11 + height/2) >>> 6 (Floor)
            
            j00 = J[0,0]
            j11 = J[1,1]
            
            # Hardware calculation simulation
            # cam_pos is Q10.6, J is Q10.6
            # product is Q20.12
            # width/2 is integer 32 (for 64x64). 
            # For 128x128, width/2 is 64. 
            # In Q20.12, 32 * 2^12. 
            # Wait, compute_jacobian used width/2 for fx calculation.
            # Here we need to implement the screen mapping carefully.
            
            def quantize_floor(val):
                # val is the float result of (cam_pos * J + offset)
                # We want to emulate (val_Q20_12) >>> 6 -> Q10.6
                # This is equivalent to floor(val * 64) / 64
                return math.floor(val * 64) / 64
            
            screen_x = cam_pos[0] * j00 + width/2
            screen_y = cam_pos[1] * j11 + height/2 # j11 is negative
            
            screen_pos = np.array([quantize_floor(screen_x), quantize_floor(screen_y)])
            
            # Compute Covariance 2D
            cov2d, cov2d_unclamped = compute_cov2d(cov3d, view_matrix, J)
            
            # BBox Calculation
            # Use UNCLAMPED Cov2D to match Hardware Stage 5 internal logic
            min_x, max_x, min_y, max_y, rad_x, rad_y = compute_bbox(screen_pos, cov2d_unclamped, width, height)
            
            bbox = (min_x, max_x, min_y, max_y, rad_x, rad_y)
            
            # Conic Calculation
            # Use CLAMPED Cov2D to match Hardware Stage 6 input
            conic = compute_conic(cov2d)

            
            # Write Golden Data (Delayed)
            f_cov2d.write("".join([to_hex_16(Q_MAIN, x) for x in cov2d.flatten()]) + "\n")
            f_conic.write("".join([to_hex_16(Q_CONIC, x) for x in conic]) + "\n")
            
            # Write BBox & Screen Pos
            # Format: sx(16) sy(16) minx(16) maxx(16) miny(16) maxy(16)
            sx_h = to_hex_16(Q_MAIN, screen_pos[0])
            sy_h = to_hex_16(Q_MAIN, screen_pos[1])
            minx_h = f"{bbox[0]:04x}"
            maxx_h = f"{bbox[1]:04x}"
            miny_h = f"{bbox[2]:04x}"
            maxy_h = f"{bbox[3]:04x}"
            f_bbox.write(f"{sx_h}{sy_h}{minx_h}{maxx_h}{miny_h}{maxy_h}\n")

            # Stage Raster Param (New SRAM)
            # Layout: [Reserved(16)][Ry(16)][Rx(16)][Cy(16)][Cx(16)][Cc(16)][Cb(16)][Ca(16)] (MSB..LSB)
            # Conic A, B, C: Q4.12 -> convert to hex
            # Center X, Y: Q10.6 -> convert to hex (screen_pos)
            # Radius X, Y: Q10.6 -> convert rad_x, rad_y to Q10.6
            
            rad_x_val = bbox[4]
            rad_y_val = bbox[5]
            
            # Conic
            ca_h = to_hex_16(Q_CONIC, conic[0])
            cb_h = to_hex_16(Q_CONIC, conic[1])
            cc_h = to_hex_16(Q_CONIC, conic[2])
            
            # Center (Screen Pos is Q10.6 float modeled)
            cx_h = to_hex_16(Q_MAIN, screen_pos[0])
            cy_h = to_hex_16(Q_MAIN, screen_pos[1])
            
            # Radius (Quantize to Q10.6)
            rx_h = to_hex_16(Q_MAIN, rad_x_val)
            ry_h = to_hex_16(Q_MAIN, rad_y_val)
            
            # Reserved
            res_h = "0000"
            
            # Concatenate MSB to LSB
            # Data layout: {Reserved, Ry, Rx, Cy, Cx, Cc, Cb, Ca}
            f_param.write(f"{res_h}{ry_h}{rx_h}{cy_h}{cx_h}{cc_h}{cb_h}{ca_h}\n")
            
            processed_gaussians.append({
                'valid': True, 
                'id': g_data['id'], 
                'depth': z,
                'screen_pos': screen_pos,
                'conic': conic,
                'opacity': opacity,
                'color': color,
                'bbox': bbox
            })
            
        # Stage 7: Sort
        valid_gaussians = [g for g in processed_gaussians if g['valid']]
        valid_gaussians.sort(key=lambda x: x['depth'])
        
        for g in valid_gaussians:
            # ID (32-bit hex = 8 chars), Depth (16-bit hex = 4 chars)
            f_sort.write(f"{g['id']:08x}{to_hex_16(Q_MAIN, g['depth'])}\n")
            
        # Fill remaining slots with Fs or 0s
        for _ in range(num_gaussians - len(valid_gaussians)):
            f_sort.write(f"ffffffff{to_hex_16(Q_MAIN, 0)}\n")
            
        # Stage 8: Rasterization (Hardware Simulation)
        canvas, sram_int = simulate_accumulation_hardware_flow(valid_gaussians, width, height, s)
        
        # Write Image Data from simulated canvas
        # Use the raw integer SRAM values for exact hardware matching
        # Format: T(16) R(16) G(16) B(16) (Big Endian in file string)
        for y in range(height):
            for x in range(width):
                t_val = sram_int[y, x, 0]
                r_val = sram_int[y, x, 1]
                g_val = sram_int[y, x, 2]
                b_val = sram_int[y, x, 3]
                
                # T is Q0.8 (0-255), pad to 16-bit
                t_h = f"{t_val:04x}"
                
                # Colors are Q10.6 (signed 16-bit)
                # sram_int contains Python ints, need to mask to 16-bit
                r_h = f"{r_val & 0xFFFF:04x}"
                g_h = f"{g_val & 0xFFFF:04x}"
                b_h = f"{b_val & 0xFFFF:04x}"
                
                # Write 64-bit word: T R G B
                f_img.write(f"{t_h}{r_h}{g_h}{b_h}\n")

        
        # Save PPM (Visual Debug)
        ppm_header = f"P6\n{width} {height}\n255\n"
        with open(os.path.join(DIRS["visual"], f"render_{s:02d}.ppm"), "wb") as f_ppm:
            f_ppm.write(ppm_header.encode())
            img_uint8 = np.clip(canvas * 255, 0, 255).astype(np.uint8)
            f_ppm.write(img_uint8.tobytes())
            
        # Save PNG (Visual Debug)
        Image.fromarray(img_uint8).save(os.path.join(DIRS["visual"], f"render_{s:02d}.png"))
                
        # Close files
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
        
    print("Done.")

if __name__ == "__main__":
    generate_data()
