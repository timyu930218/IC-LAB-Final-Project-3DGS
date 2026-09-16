module Stage6_Conic #(
    parameter BIT_WIDTH = 16, 
    parameter LONG_BIT_WIDTH = 40,
    parameter FLOAT_WIDTH = 6,
    parameter PRODUCT_WIDTH = 32,
    parameter PRODUCT_WIDTH_LONG = 56,
    parameter RENDER_WIDTH = 64,
    parameter RENDER_HEIGHT = 64
)(
    input clk,
    input srst_n,
    input enable,
    input [6:0] gaussian_cnt,
    input [9*PRODUCT_WIDTH-1:0] conic_product,
    input [9*PRODUCT_WIDTH_LONG-1:0] conic_product_long,
    input [35:0] conic_quotient,

    output valid,
    
    input [4*BIT_WIDTH-1:0] conv2d,      // Q10.6: [sigma00, sigma01, sigma10, sigma11]
    output reg [3*BIT_WIDTH-1:0] conic_coeff, // Q4.12: [conic_a, conic_b, conic_c]
    output reg signed [9*BIT_WIDTH-1:0] conic_multiplicand,
    output reg signed [9*BIT_WIDTH-1:0] conic_multiplier,

    output reg signed [9*LONG_BIT_WIDTH-1:0] conic_multiplicand_long,
    output reg signed [9*BIT_WIDTH-1:0] conic_multiplier_long,

    output reg [35:0] conic_dividend,
    output reg [35:0] conic_divisor, 

    // SRAM Interface (Raster Parameter), Write Only 
    output reg [127:0] sram_wdata_raster,
    output reg [5:0] sram_waddr_raster,
    output reg [15:0] sram_wordmask_raster,
    output reg         sram_wen_raster

);

    //========================
    // DECLARE
    //========================
    localparam IDLE = 3'b000;
    localparam CALC_DET = 3'b001;
    localparam DIV = 3'b010;
    localparam MULT = 3'b011;
    localparam DONE = 3'b100;

    reg [2:0] state, state_n;
    reg [3:0] calc_det_count, calc_det_count_n;
    reg [3:0] div_count, div_count_n;
    reg [3:0] mult_count, mult_count_n;
    reg signed [PRODUCT_WIDTH_LONG-1:0] temp_a, temp_b, temp_c;

    reg sign, sign_n;
    reg [2*BIT_WIDTH-1:0] det_abs, det_abs_n;
    reg [3*BIT_WIDTH-1:0] conic_coeff_n; 

    wire signed [BIT_WIDTH-1:0] sigma00, sigma01, sigma10, sigma11;
    
    // Det Calculation
    // sigma: Q10.6
    // mult: Q20.12
    reg signed [2*BIT_WIDTH-1:0] det_q; // 32-bit Q20.12
    reg signed [2*BIT_WIDTH-1:0] det_q_n;
    
    // Det Inv
    // Target: Q1.23 (24-bit)
    // Formula: det_inv = (2^35) / det_q
    // We need 64-bit to hold 2^35
    reg signed [35:0] det_inv_q; // Q1.23 (actually need more bits for calc)
    reg signed [35:0] det_inv_q_n;

    // Conic Coeffs
    // Q10.6 * Q1.23 = Q11.29
    // Target Q4.12
    // Shift right by 17
    reg signed [BIT_WIDTH-1:0] conic_a, conic_b, conic_c;
    reg signed [BIT_WIDTH-1:0] conic_a_n, conic_b_n, conic_c_n;

    // Unpacking
    // Note: Python output order in file is 00, 01, 10, 11 (row major)
    assign sigma00 = conv2d[4*BIT_WIDTH-1:3*BIT_WIDTH];
    assign sigma01 = conv2d[3*BIT_WIDTH-1:2*BIT_WIDTH]; // same as sigma10 usually
    assign sigma10 = conv2d[2*BIT_WIDTH-1:BIT_WIDTH];
    assign sigma11 = conv2d[BIT_WIDTH-1:0];

    //========================
    // DATAPATH
    //========================
    // Multipliers for Det
    /*
    wire signed [2*BIT_WIDTH-1:0] mult1 = sigma00 * sigma11; // Q20.12
    wire signed [2*BIT_WIDTH-1:0] mult2 = sigma01 * sigma01; // Q20.12
    */

    wire signed [35:0] neg_det_inv_q = -det_inv_q;

    always@* begin
        conic_multiplicand = {sigma00, sigma01, 112'b0};
        conic_multiplier = {sigma11, sigma01, 112'b0};
        
        // Fix 1: Properly sign-extend det_inv_q (36 to 40 bits)
        // Fix 2: For Conic B, use (-det_inv) * sigma01 instead of det_inv * (-sigma01)
        //        This prevents 16-bit overflow when sigma01 = -32768.
        
        conic_multiplicand_long = { {4{det_inv_q[35]}}, det_inv_q, 
                                    {4{neg_det_inv_q[35]}}, neg_det_inv_q, 
                                    {4{det_inv_q[35]}}, det_inv_q, 
                                    240'b0};
                                    
        conic_multiplier_long = {sigma11, sigma01, sigma00, 96'b0};

        conic_dividend = 36'h8_0000_0000 + (det_abs >>> 1);
        conic_divisor = {4'b0, det_abs};

    end


    //========================
    // FSM 
    //========================

    // Sequential Logic
    always @(posedge clk) begin
        if (!srst_n) begin
            state <= IDLE;
            calc_det_count <= 0;
            div_count <= 0;
            mult_count <= 0;
            det_q <= 0;
            det_inv_q <= 0;
            conic_a <= 0;
            conic_b <= 0;
            conic_c <= 0;
            conic_coeff <= 0;
        end else begin
            state <= state_n;
            calc_det_count <= calc_det_count_n;
            div_count <= div_count_n;
            mult_count <= mult_count_n;
            det_q <= det_q_n;
            det_inv_q <= det_inv_q_n;
            sign <= sign_n;
            det_abs <= det_abs_n;
            conic_a <= conic_a_n;
            conic_b <= conic_b_n;
            conic_c <= conic_c_n;
            conic_coeff <= conic_coeff_n;
        end
    end

    always @(*) begin
        state_n = state;
        calc_det_count_n = calc_det_count;
        div_count_n = div_count;
        mult_count_n = mult_count;
        det_q_n = det_q;
        det_inv_q_n = det_inv_q;
        sign_n = sign;
        det_abs_n = det_abs;
        conic_a_n = conic_a;
        conic_b_n = conic_b;
        conic_c_n = conic_c;
        conic_coeff_n = conic_coeff;
        sram_waddr_raster = 0;
        sram_wdata_raster = 0;
        sram_wordmask_raster = 16'b0;
        sram_wen_raster = 1;
        case(state)
            IDLE: begin
                if (enable) state_n = CALC_DET;
            end
            CALC_DET: begin
                case (calc_det_count)
                    0: begin
                        calc_det_count_n = calc_det_count + 1;
                        state_n = CALC_DET;
                    end

                    1: begin
                        det_q_n = conic_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH] - conic_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH];
                        calc_det_count_n = 0;
                        state_n = DIV;
                    end
                    default:;
                endcase
            end
            DIV: begin
                case (div_count)
                    0: begin
                        sign_n = (det_q < 0);
                        det_abs_n = (det_q < 0) ? -det_q : det_q;
                        
                        if (det_abs_n == 0) det_abs_n = 1;       

                        // det_inv = 1.0 / det
                        // Use unsigned division with Rounding
                        // (Numerator + Denominator/2) / Denominator
                        // 1<<35 = 0x8_0000_0000 (unsigned)

                        div_count_n = div_count + 1;
                        state_n = DIV;
                    end

                    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11: begin
                        div_count_n = div_count + 1;
                        state_n = DIV;
                    end

                    12: begin

                        if (sign) det_inv_q_n = -conic_quotient;
                        else det_inv_q_n = conic_quotient;

                        div_count_n = 0;
                        state_n = MULT;
                    end

                    default:;

                endcase
            end
            MULT: begin
                case (mult_count)
                    0, 1, 2: begin
                        mult_count_n = mult_count + 1;
                        state_n = MULT;
                    end
                    3: begin
                        // Saturation Logic for Conic A
                        temp_a = ( $signed(conic_product_long[9*PRODUCT_WIDTH_LONG-1:8*PRODUCT_WIDTH_LONG]) + 32'sd65536) >>> 17;
                        if (temp_a > 32767) conic_a_n = 32767;
                        else if (temp_a < -32768) conic_a_n = -32768;
                        else conic_a_n = temp_a[15:0];

                        // Saturation Logic for Conic B
                        temp_b = ( $signed(conic_product_long[8*PRODUCT_WIDTH_LONG-1:7*PRODUCT_WIDTH_LONG]) + 32'sd65536) >>> 17;
                        if (temp_b > 32767) conic_b_n = 32767;
                        else if (temp_b < -32768) conic_b_n = -32768;
                        else conic_b_n = temp_b[15:0];

                        // Saturation Logic for Conic C
                        temp_c = ( $signed(conic_product_long[7*PRODUCT_WIDTH_LONG-1:6*PRODUCT_WIDTH_LONG])  + 32'sd65536) >>> 17;
                        if (temp_c > 32767) conic_c_n = 32767;
                        else if (temp_c < -32768) conic_c_n = -32768;
                        else conic_c_n = temp_c[15:0];

                        mult_count_n = mult_count + 1;
                        state_n = MULT; 
                    end
                    4: begin
                        mult_count_n = mult_count + 1;
                        conic_coeff_n = {conic_a, conic_b, conic_c}; 
                        state_n = MULT; 
                    end 
                    5: begin 
                        mult_count_n = 0;
                        state_n = DONE;
                        sram_waddr_raster = gaussian_cnt;
                        sram_wdata_raster = {80'b0, conic_c, conic_b, conic_a};
                        sram_wordmask_raster = {10'b1111111111,6'b0 };
                        sram_wen_raster = 0;
                    end 
                    default:;
                endcase
            end
            DONE: begin
                state_n = IDLE;
            end
            default:;
        endcase
    end

    assign valid = (state == DONE);

endmodule