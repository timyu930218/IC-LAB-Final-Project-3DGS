module Stage8_Render #(
    parameter BIT_WIDTH = 16, 
    parameter LONG_BIT_WIDTH = 40,
    parameter PRODUCT_WIDTH = 32,
    parameter PRODUCT_WIDTH_LONG = 56,
    parameter RENDER_WIDTH = 128,
    parameter RENDER_HEIGHT = 128,
    parameter RENDER_WIDTH_LOG = 7
)(
    input clk,
    input srst_n,
    input enable,    
    input [6:0] gaussian_num, 
    input [9*PRODUCT_WIDTH-1:0] render_product,
    input [9*PRODUCT_WIDTH_LONG-1:0] render_product_long,
    
    output valid,

    // SRAM Interface (Input Data) - 64-bit
    
    input  [63:0] sram_rdata_input,
    //output reg [63:0] sram_wdata_input,
    output reg [15:0]  sram_raddr_input,
    //output reg [15:0]  sram_waddr_input,

    // SRAM Interface (Output Image) - 64-bit
    input  [63:0] sram_rdata_img,
    output reg [63:0] sram_wdata_img,
    output reg [15:0]  sram_raddr_img,
    output reg [15:0]  sram_waddr_img,
    output reg         sram_wen_img, 

    // SRAM Interface (Output Temp) - 64-bit
    //input  [63:0] sram_rdata_temp,
    //output reg [63:0] sram_wdata_temp,
    //output reg [15:0]  sram_raddr_temp,
    //output reg [15:0]  sram_waddr_temp,
    //output reg         sram_wen_temp, 

    // SRAM Interface (Raster)
    input  [127:0] sram_rdata_raster,
    output reg [5:0] sram_raddr_raster,

    // SRAM Interface (Sort)
    input  [31:0] sram_rdata_sort,
    output reg [5:0] sram_raddr_sort,

    // input [6*BIT_WIDTH-1:0] bbox_info, 
    // input [3*BIT_WIDTH-1:0] conic_coeff,

    output reg signed [9*BIT_WIDTH-1:0] render_multiplicand,
    output reg signed [9*BIT_WIDTH-1:0] render_multiplier,

    output reg signed [9*LONG_BIT_WIDTH-1:0] render_multiplicand_long,
    output reg signed [9*BIT_WIDTH-1:0] render_multiplier_long
);

    //========================
    // DECLARE
    //========================

    localparam IDLE = 3'd0,
               FETCH_SORT = 3'd1,
               FETCH_PARAM = 3'd2, 
               CALC = 3'd3,
               RENDER = 3'd4,
               DONE = 3'd5;

    localparam PAD = RENDER_WIDTH_LOG - 4;
    reg [2:0] state, state_n;
    reg [2:0] read_count, read_count_n;
    reg [3:0] calc_count, calc_count_n;
    reg [3:0] render_count, render_count_n;
    reg [6:0] gaussian_cnt, gaussian_cnt_n;
    reg [5:0] current_gaussian_id;

    //stage 0
    wire [RENDER_WIDTH_LOG-1:0] s0_x, s0_y;
    reg [2*RENDER_WIDTH_LOG-1:0] s0_pixel_count; 
    reg s0_valid;
    reg s0_done;
    assign s0_x = s0_pixel_count[RENDER_WIDTH_LOG-1 : 0];
    assign s0_y = s0_pixel_count[2*RENDER_WIDTH_LOG-1 : RENDER_WIDTH_LOG];

    //stage 1
    reg [2*RENDER_WIDTH_LOG-1:0] s1_pixel_count;
    reg s1_valid;
    reg s1_done;
    reg signed [31:0] s1_dx_sq; // Q20.12
    reg signed [31:0] s1_dy_sq; // Q20.12
    reg signed [31:0] s1_dx_dy; // Q20.12

    //stage 2
    reg [2*RENDER_WIDTH_LOG-1:0] s2_pixel_count;  
    reg s2_valid;
    reg s2_done;
    reg signed [31:0] s2_dx_sq; // Q20.12
    reg signed [31:0] s2_dy_sq; // Q20.12
    reg signed [31:0] s2_dx_dy; // Q20.12

    //stage 3
    reg [2*RENDER_WIDTH_LOG-1:0] s3_pixel_count;
    reg s3_valid;
    reg s3_done;
    reg [BIT_WIDTH-1:0] s3_old_color_t;
    reg [BIT_WIDTH-1:0] s3_old_color_r;
    reg [BIT_WIDTH-1:0] s3_old_color_g;
    reg [BIT_WIDTH-1:0] s3_old_color_b;

    //stage 4
    reg [2*RENDER_WIDTH_LOG-1:0] s4_pixel_count;
    reg s4_valid;
    reg s4_done;
    reg [BIT_WIDTH-1:0] s4_old_color_t;
    reg [BIT_WIDTH-1:0] s4_old_color_r;
    reg [BIT_WIDTH-1:0] s4_old_color_g;
    reg [BIT_WIDTH-1:0] s4_old_color_b;

    //stage 5
    reg [2*RENDER_WIDTH_LOG-1:0] s5_pixel_count;
    reg s5_valid;
    reg s5_done;
    reg signed [47:0] s5_term_a; 
    reg signed [47:0] s5_term_b; // needs * 2
    reg signed [47:0] s5_term_c;
    reg [BIT_WIDTH-1:0] s5_old_color_t;
    reg [BIT_WIDTH-1:0] s5_old_color_r;
    reg [BIT_WIDTH-1:0] s5_old_color_g;
    reg [BIT_WIDTH-1:0] s5_old_color_b;

    //stage 6
    reg [2*RENDER_WIDTH_LOG-1:0] s6_pixel_count;
    reg s6_valid;
    reg s6_done;
    reg signed [47:0] s6_term_a; 
    reg signed [47:0] s6_term_b; // needs * 2
    reg signed [47:0] s6_term_c;
    reg [BIT_WIDTH-1:0] s6_old_color_t;
    reg [BIT_WIDTH-1:0] s6_old_color_r;
    reg [BIT_WIDTH-1:0] s6_old_color_g;
    reg [BIT_WIDTH-1:0] s6_old_color_b;

    //stage 7
    reg [2*RENDER_WIDTH_LOG-1:0] s7_pixel_count;
    reg s7_valid;
    reg s7_done;
    reg [BIT_WIDTH-1:0] s7_old_color_t;
    reg [BIT_WIDTH-1:0] s7_old_color_r;
    reg [BIT_WIDTH-1:0] s7_old_color_g;
    reg [BIT_WIDTH-1:0] s7_old_color_b;

    //stage 8
    reg [2*RENDER_WIDTH_LOG-1:0] s8_pixel_count;
    reg s8_valid;
    reg s8_done;
    reg signed [31:0] s8_alpha_prod;
    reg signed [31:0] s8_alpha_rounded;
    reg [BIT_WIDTH-1:0] s8_alpha; 
    reg [BIT_WIDTH-1:0] s8_old_color_t;
    reg [BIT_WIDTH-1:0] s8_old_color_r;
    reg [BIT_WIDTH-1:0] s8_old_color_g;
    reg [BIT_WIDTH-1:0] s8_old_color_b;

    //stage 9
    reg [2*RENDER_WIDTH_LOG-1:0] s9_pixel_count;
    reg s9_valid;
    reg s9_done;
    reg [BIT_WIDTH-1:0] s9_alpha; 
    reg [BIT_WIDTH-1:0] s9_old_color_t;
    reg [BIT_WIDTH-1:0] s9_old_color_r;
    reg [BIT_WIDTH-1:0] s9_old_color_g;
    reg [BIT_WIDTH-1:0] s9_old_color_b;

    //stage 10
    reg [2*RENDER_WIDTH_LOG-1:0] s10_pixel_count;
    reg s10_valid;
    reg s10_done;
    reg [BIT_WIDTH-1:0] s10_alpha; 
    reg [BIT_WIDTH-1:0] s10_old_color_t;
    reg [BIT_WIDTH-1:0] s10_old_color_r;
    reg [BIT_WIDTH-1:0] s10_old_color_g;
    reg [BIT_WIDTH-1:0] s10_old_color_b;
    reg [BIT_WIDTH-1:0] s10_contribution_alpha;

    //stage 11
    reg [2*RENDER_WIDTH_LOG-1:0] s11_pixel_count;
    reg s11_valid;
    reg s11_done;
    reg signed [31:0] s11_alpha_rounded;
    reg [BIT_WIDTH-1:0] s11_alpha;
    reg [BIT_WIDTH-1:0] s11_old_color_t;
    reg [BIT_WIDTH-1:0] s11_old_color_r;
    reg [BIT_WIDTH-1:0] s11_old_color_g;
    reg [BIT_WIDTH-1:0] s11_old_color_b;
    reg [BIT_WIDTH-1:0] s11_contribution_alpha;

    //stage 12
    reg [2*RENDER_WIDTH_LOG-1:0] s12_pixel_count;
    reg s12_valid;
    reg s12_done;
    reg [2*BIT_WIDTH-1:0] s12_color_final_r_q, s12_color_final_g_q, s12_color_final_b_q ; 
    reg [BIT_WIDTH-1:0] s12_alpha;
    reg [BIT_WIDTH-1:0] s12_old_color_t;
    reg [BIT_WIDTH-1:0] s12_old_color_r;
    reg [BIT_WIDTH-1:0] s12_old_color_g;
    reg [BIT_WIDTH-1:0] s12_old_color_b;

    //stage 13
    reg [2*RENDER_WIDTH_LOG-1:0] s13_pixel_count;
    wire [RENDER_WIDTH_LOG-1:0] s13_x, s13_y;
    reg s13_valid;
    reg s13_done;
    assign s13_x = s13_pixel_count[RENDER_WIDTH_LOG-1:0];
    assign s13_y = s13_pixel_count[2*RENDER_WIDTH_LOG-1:RENDER_WIDTH_LOG];
    reg [BIT_WIDTH-1:0] s13_alpha;
    reg [BIT_WIDTH-1:0] s13_old_color_r;
    reg [BIT_WIDTH-1:0] s13_old_color_g;
    reg [BIT_WIDTH-1:0] s13_old_color_b;
    reg [BIT_WIDTH-1:0] s13_old_color_t;
    reg [2*BIT_WIDTH-1:0] s13_color_final_r_q, s13_color_final_g_q, s13_color_final_b_q ;

    //stage 13

    
    // color signal , read from input sram
    reg [BIT_WIDTH-1:0] old_color_r, old_color_g, old_color_b, old_color_t ; // t is transparency

    reg [BIT_WIDTH-1:0] color_r, color_g, color_b, color_t ; // t is transparency
    reg [BIT_WIDTH-1:0] color_r_n, color_g_n, color_b_n, color_t_n ; // t is transparency

    // conic variable, read from input directly
    wire signed [BIT_WIDTH-1:0] conic_a, conic_b, conic_c;
    // assign conic_a = conic_coeff[3*BIT_WIDTH-1:2*BIT_WIDTH];
    // assign conic_b = conic_coeff[2*BIT_WIDTH-1:BIT_WIDTH];
    // assign conic_c = conic_coeff[BIT_WIDTH-1:0];
    assign conic_a = sram_rdata_raster[BIT_WIDTH-1:0];
    assign conic_b = sram_rdata_raster[2*BIT_WIDTH-1:BIT_WIDTH];
    assign conic_c = sram_rdata_raster[3*BIT_WIDTH-1:2*BIT_WIDTH];

    // bbox information, read from input directly
    wire signed [RENDER_WIDTH_LOG-1:0] bbox_x_min, bbox_x_max, bbox_y_min, bbox_y_max;
    wire signed [BIT_WIDTH-1:0] rad_x, rad_y;
    wire signed [BIT_WIDTH-1:0] center_x, center_y;
    //assign bbox_x_min = bbox_info[4*BIT_WIDTH-1:3*BIT_WIDTH];
    //assign bbox_x_max = bbox_info[3*BIT_WIDTH-1:2*BIT_WIDTH];
    //assign bbox_y_min = bbox_info[2*BIT_WIDTH-1:BIT_WIDTH];
    //assign bbox_y_max = bbox_info[BIT_WIDTH-1:0];
    assign rad_x = sram_rdata_raster[6*BIT_WIDTH-1:5*BIT_WIDTH];
    assign rad_y = sram_rdata_raster[7*BIT_WIDTH-1:6*BIT_WIDTH];
    //assign bbox_x_min = ((center_x-rad_x)<0) ? 0 : (center_x - rad_x) >>> 6;
    //assign bbox_x_max = (((center_x+rad_x)>>>6)>63) ? 63 : (center_x + rad_x) >>> 6;
    //assign bbox_y_min = ((center_y-rad_y)<0) ? 0 : (center_y - rad_y) >>> 6;
    //assign bbox_y_max = (((center_y+rad_y)>>>6)>63) ? 63 : (center_y + rad_y) >>> 6;

    assign bbox_x_min = ((center_x-rad_x)<0) ? 0 : (center_x - rad_x) >>> 6;
    assign bbox_y_min = ((center_y-rad_y)<0) ? 0 : (center_y - rad_y) >>> 6;
    wire signed [BIT_WIDTH-1:0] max_x_q, max_y_q;
    assign max_x_q = center_x + rad_x;
    assign max_y_q = center_y + rad_y;
    assign bbox_x_max = ((max_x_q >>> 6) >= RENDER_WIDTH) ? (RENDER_WIDTH-1) : (max_x_q >>> 6);
    assign bbox_y_max = ((max_y_q >>> 6) >= RENDER_HEIGHT)? (RENDER_HEIGHT-1): (max_y_q >>> 6);


    //assign center_x = bbox_info[6*BIT_WIDTH-1:5*BIT_WIDTH];
    //assign center_y = bbox_info[5*BIT_WIDTH-1:4*BIT_WIDTH];
    assign center_x = sram_rdata_raster[4*BIT_WIDTH-1:3*BIT_WIDTH];
    assign center_y = sram_rdata_raster[5*BIT_WIDTH-1:4*BIT_WIDTH];

    wire signed [BIT_WIDTH-1:0] delta_x , delta_y;
    wire signed [3*BIT_WIDTH-1:0] P_q;
    wire signed [BIT_WIDTH-1:0] P;
    wire signed [10:0] Power;

    wire [BIT_WIDTH-1:0] exp_lut_out;

    // Fix for Pipeline Hazard:
    // P is derived from Stage 6. We must pipeline its validity check to Stage 8.
    wire p_is_valid_s6 = (P <= 0);
    reg s7_p_valid, s8_p_valid;

    //========================
    // DATAPATH
    //========================

    // 1. Delta Calculation
    // x, y are integers (0..63). Convert to Q10.6 by shifting left 6.

    assign delta_x = {{PAD{1'b0}}, s0_x, 6'b0} - center_x; 
    assign delta_y = {{PAD{1'b0}}, s0_y, 6'b0} - center_y;



    // 2. Power Calculation
    // P = -0.5 * (A*dx^2 + B*dx*dy + C*dy^2)

/*
    reg signed [31:0] dx_sq, dx_sq_n; // Q20.12
    reg signed [31:0] dy_sq, dy_sq_n; // Q20.12
    reg signed [31:0] dx_dy, dx_dy_n; // Q20.12
*/
    // Intermediate terms (Q24.24) -> 48 bits (16+32)

/*
    reg signed [47:0] term_a, term_a_n; 
    reg signed [47:0] term_b, term_b_n; // needs * 2
    reg signed [47:0] term_c, term_c_n;
*/

    wire signed [47:0] sum_terms = s6_term_a + (s6_term_b <<< 1) + s6_term_c;

    // Multiply by -0.5 is same as negate and shift right by 1
    // Q24.24 >>> 1 -> Q24.24 (magnitude halved)
    
    wire signed [47:0] power_temp = -(sum_terms >>> 1);
    
    // Clamp to Q4.12 Range (-8.0 to 7.999)
    // Min Q4.12 is -32768. 
    wire signed [47:0] power_shifted = (power_temp + (1<<11)) >>> 12;
    assign P = (power_shifted < -48'sd32768) ? 16'h8000 : 
               (power_shifted > 48'sd32767) ? 16'h7FFF :
               power_shifted[15:0]; 


    // 3. LUT Interface
    // P is Q4.12 negative.
    wire signed [15:0] P_abs = (P == 16'h8000) ? 16'h7FFF : (P < 0 ? -P : P);
    
    wire [11:0] P_idx = P_abs[15:4]; 
    
    assign Power = (P_idx > 12'd2047) ? 11'd2047 : P_idx[10:0];

    // explut
    exp_lut exp_lut_inst(
        .clk(clk),
        .addr(Power),
        .data(exp_lut_out)
    );

    // 4. Alpha Calculation
    // alpha = opacity * exp_out
    // color_t (opacity) is Q10.6, exp_lut_out is Q8.8 -> Product Q18.14
    /*
    reg signed [31:0] alpha_prod;
    reg signed [31:0] alpha_rounded, alpha_rounded_n;

    
    reg [2*BIT_WIDTH-1:0] color_final_r_q, color_final_g_q, color_final_b_q ; 
    reg [2*BIT_WIDTH-1:0] color_final_r_q_n, color_final_g_q_n, color_final_b_q_n ; 
    */

    wire [BIT_WIDTH-1:0] color_final_r, color_final_g, color_final_b;
    wire [BIT_WIDTH-1:0] color_final_t;

    wire [2*BIT_WIDTH-1:0] color_mid_r = (s13_color_final_r_q + (1<<5)) >>> 6;
    wire [2*BIT_WIDTH-1:0] color_mid_g = (s13_color_final_g_q + (1<<5)) >>> 6;
    wire [2*BIT_WIDTH-1:0] color_mid_b = (s13_color_final_b_q + (1<<5)) >>> 6;

    // Final rounding to Q10.6 (Scale 64) = Q8.8 >>> 2
    assign color_final_r = (color_mid_r + (1<<1)) >>> 2;
    assign color_final_g = (color_mid_g + (1<<1)) >>> 2;
    assign color_final_b = (color_mid_b + (1<<1)) >>> 2;

    wire signed [15:0] s12_alpha_check = 256 - s12_alpha;
    assign color_final_t = (s13_alpha == 0 && s13_old_color_t == 256)? 256 : render_product[1*PRODUCT_WIDTH-1:0*PRODUCT_WIDTH] >>> 8 ; 

    //========================
    // FSM
    //========================

    always @(posedge clk) begin
        if (~srst_n) begin
            state <= IDLE;
            read_count <= 0;
            color_r <= 16'd0;
            color_g <= 16'd0;
            color_b <= 16'd0;
            color_t <= 16'd0;
            gaussian_cnt <= 0;
        end else begin
            state <= state_n;
            read_count <= read_count_n;
            color_r <= color_r_n;
            color_g <= color_g_n;
            color_b <= color_b_n;
            color_t <= color_t_n;
            gaussian_cnt <= gaussian_cnt_n;
        end
    end

    always@(posedge clk) begin
        if (~srst_n) begin
            s0_pixel_count <= 0;
            s0_valid <= 0;
            s0_done <= 0;
        end else begin

            //s0_stage

            
            if (gaussian_cnt == 0) begin
                if (state == CALC) begin
                    if (s0_pixel_count < RENDER_WIDTH*RENDER_HEIGHT- 1 && 
                        s0_done == 0 && s1_done == 0 && s2_done == 0 && s3_done == 0 && 
                        s4_done == 0 && s5_done == 0 && s6_done == 0 && s7_done == 0 &&
                        s8_done == 0 && s9_done == 0 && s10_done == 0 && s11_done == 0 &&
                        s12_done == 0 && s13_done == 0 ) begin
                        if (s0_valid) s0_pixel_count <= s0_pixel_count + 1;
                        else s0_pixel_count <= 0;

                        s0_valid <= 1;
                        s0_done <= 0;
                    end else if(s0_pixel_count == RENDER_WIDTH*RENDER_HEIGHT-1) begin
                        s0_pixel_count <= 0;
                        s0_valid <= 0;
                        s0_done <= 1;
                    end else begin
                        s0_pixel_count <= 0;
                        s0_valid <= 0;
                        s0_done <= 0;
                    end
                end else begin
                    s0_pixel_count <= 0;
                    s0_valid <= 0;
                    s0_done <= 0;
                end
            end else begin
                if (state == CALC) begin
                    if (s0_pixel_count[2*RENDER_WIDTH_LOG-1:1*RENDER_WIDTH_LOG] <= bbox_y_max && 
                        s0_done == 0 && s1_done == 0 && s2_done == 0 && s3_done == 0 && 
                        s4_done == 0 && s5_done == 0 && s6_done == 0 && s7_done == 0 &&
                        s8_done == 0 && s9_done == 0 && s10_done == 0 && s11_done == 0 &&
                        s12_done == 0 && s13_done == 0 ) begin
                        if (s0_pixel_count[1*RENDER_WIDTH_LOG-1:0] < bbox_x_max) begin
                            if (s0_valid) s0_pixel_count <= s0_pixel_count + 1;
                            else s0_pixel_count <= {bbox_y_min, bbox_x_min};

                            s0_valid <= 1;
                            s0_done <= 0;
                        end else if (s0_pixel_count == {bbox_y_max, bbox_x_max}) begin
                            s0_pixel_count <= 0;
                            s0_valid <= 0;
                            s0_done <= 1;
                        end else begin
                            s0_pixel_count <= {s0_pixel_count[2*RENDER_WIDTH_LOG-1:1*RENDER_WIDTH_LOG] + 1, bbox_x_min};
                            s0_valid <= 1;
                            s0_done <= 0;
                        end
                    end else begin
                        s0_pixel_count <= 0;
                        s0_valid <= 0;
                        s0_done <= 0;
                    end
                end else begin
                    s0_pixel_count <= 0;
                    s0_valid <= 0;
                    s0_done <= 0;
                end
            end
            
                //old version
            
            /*
            if (state == CALC) begin
                if (s0_pixel_count < RENDER_WIDTH*RENDER_HEIGHT- 1 && s0_done == 0) begin
                    if (s0_valid) s0_pixel_count <= s0_pixel_count + 1;
                    else s0_pixel_count <= 0;

                    s0_valid <= 1;
                    s0_done <= 0;
                end else if(s0_pixel_count == RENDER_WIDTH*RENDER_HEIGHT-1) begin
                    s0_pixel_count <= 0;
                    s0_valid <= 0;
                    s0_done <= 1;
                end else begin
                    s0_pixel_count <= 0;
                    s0_valid <= 0;
                    s0_done <= 0;
                end
            end else begin
                s0_pixel_count <= 0;
                s0_valid <= 0;
                s0_done <= 0;
            end
            */

            //s1_stage
            s1_pixel_count <= s0_pixel_count;
            s1_valid <= s0_valid;
            s1_done <= s0_done;

            //s2_stage
            s2_pixel_count <= s1_pixel_count;
            s2_valid <= s1_valid;
            s2_done <= s1_done;

            s2_dx_sq <= s1_dx_sq;
            s2_dy_sq <= s1_dy_sq;
            s2_dx_dy <= s1_dx_dy;

            //s3_stage
            s3_pixel_count <= s2_pixel_count;
            s3_valid <= s2_valid;
            s3_done <= s2_done;


            //s4_stage
            s4_pixel_count <= s3_pixel_count;
            s4_valid <= s3_valid;
            s4_done <= s3_done;
            s4_old_color_t <= s3_old_color_t;
            s4_old_color_r <= s3_old_color_r;
            s4_old_color_g <= s3_old_color_g;
            s4_old_color_b <= s3_old_color_b;


            //s5_stage
            s5_pixel_count <= s4_pixel_count;
            s5_valid <= s4_valid;
            s5_done <= s4_done;
            s5_old_color_t <= s4_old_color_t;
            s5_old_color_r <= s4_old_color_r;
            s5_old_color_g <= s4_old_color_g;
            s5_old_color_b <= s4_old_color_b;

            //s6_stage
            s6_pixel_count <= s5_pixel_count;
            s6_valid <= s5_valid;
            s6_done <= s5_done;
            s6_old_color_t <= s5_old_color_t;
            s6_old_color_r <= s5_old_color_r;
            s6_old_color_g <= s5_old_color_g;
            s6_old_color_b <= s5_old_color_b;
            s6_term_a <= s5_term_a;
            s6_term_b <= s5_term_b;
            s6_term_c <= s5_term_c;

            //s7_stage
            s7_pixel_count <= s6_pixel_count;
            s7_valid <= s6_valid;
            s7_done <= s6_done;
            s7_old_color_t <= s6_old_color_t;
            s7_old_color_r <= s6_old_color_r;
            s7_old_color_g <= s6_old_color_g;
            s7_old_color_b <= s6_old_color_b;

            //s8_stage
            s8_pixel_count <= s7_pixel_count;
            s8_valid <= s7_valid;
            s8_done <= s7_done;
            s8_old_color_t <= s7_old_color_t;
            s8_old_color_r <= s7_old_color_r;
            s8_old_color_g <= s7_old_color_g;
            s8_old_color_b <= s7_old_color_b;

            //s9_stage
            s9_pixel_count <= s8_pixel_count;
            s9_valid <= s8_valid;
            s9_done <= s8_done;
            s9_old_color_t <= s8_old_color_t;
            s9_old_color_r <= s8_old_color_r;
            s9_old_color_g <= s8_old_color_g;
            s9_old_color_b <= s8_old_color_b;
            s9_alpha <= s8_alpha;

            //s10_stage
            s10_pixel_count <= s9_pixel_count;
            s10_valid <= s9_valid;
            s10_done <= s9_done;
            s10_old_color_t <= s9_old_color_t;
            s10_old_color_r <= s9_old_color_r;
            s10_old_color_g <= s9_old_color_g;
            s10_old_color_b <= s9_old_color_b;
            s10_alpha <= s9_alpha;

            //s11_stage
            s11_pixel_count <= s10_pixel_count;
            s11_valid <= s10_valid;
            s11_done <= s10_done;
            s11_old_color_t <= s10_old_color_t;
            s11_old_color_r <= s10_old_color_r;
            s11_old_color_g <= s10_old_color_g;
            s11_old_color_b <= s10_old_color_b;
            s11_alpha <= s10_alpha;
            s11_contribution_alpha <= s10_contribution_alpha;

            //s12_stage
            s12_pixel_count <= s11_pixel_count;
            s12_valid <= s11_valid;
            s12_done <= s11_done;
            s12_alpha <= s11_alpha;
            s12_old_color_t <= s11_old_color_t;
            s12_old_color_r <= s11_old_color_r;
            s12_old_color_g <= s11_old_color_g;
            s12_old_color_b <= s11_old_color_b;

            // Pipeline P valid signal
            // s6 -> s7
            if (s6_valid) s7_p_valid <= p_is_valid_s6; 
            
            // s7 -> s8
            s8_p_valid <= s7_p_valid;

            //s13_stage
            s13_pixel_count <= s12_pixel_count;
            s13_valid <= s12_valid;
            s13_done <= s12_done;
            s13_alpha <= s12_alpha;

            s13_color_final_r_q <= s12_color_final_r_q;
            s13_color_final_g_q <= s12_color_final_g_q;
            s13_color_final_b_q <= s12_color_final_b_q;
            s13_old_color_r <= s12_old_color_r;
            s13_old_color_g <= s12_old_color_g;
            s13_old_color_b <= s12_old_color_b;
            s13_old_color_t <= s12_old_color_t;

        end
    end

    always@* begin
        render_multiplicand = 0;
        render_multiplier = 0;
        render_multiplicand_long = 0;
        render_multiplier_long = 0;
        s1_dx_sq = 0;
        s1_dy_sq = 0;
        s1_dx_dy = 0;
        s5_term_a = 0;
        s5_term_b = 0;
        s5_term_c = 0;
        s8_alpha_prod = 0;
        s8_alpha_rounded = 0;
        s8_alpha = 0;
        s12_color_final_r_q = 0;
        s12_color_final_g_q = 0;
        s12_color_final_b_q = 0;

        sram_waddr_img = s13_pixel_count;
        //sram_waddr_temp = s13_pixel_count;
        sram_wen_img = 1;
        sram_wdata_img = 0;
        //sram_wen_temp = 1;
        //sram_wdata_temp = 0;
        sram_raddr_img = s0_pixel_count;
        // sram_raddr_temp = s0_pixel_count;

        s3_old_color_b = 0;
        s3_old_color_g = 0;
        s3_old_color_r = 0;
        s3_old_color_t = 0;

        s10_contribution_alpha = 0;

        if (s0_valid) begin
            render_multiplicand[9*BIT_WIDTH-1:6*BIT_WIDTH] = {delta_x, delta_y, delta_x};
            render_multiplier[9*BIT_WIDTH-1:6*BIT_WIDTH] = {delta_x, delta_y, delta_y};   
        end   

        if (s1_valid) begin
            s1_dx_sq = render_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH];
            s1_dy_sq = render_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH];
            s1_dx_dy = render_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH];
        end


        if (s2_valid) begin
            render_multiplicand_long = {s2_dx_sq, 8'b0, s2_dx_dy, 8'b0, s2_dy_sq, 8'b0, 240'b0};
            render_multiplier_long = {conic_a, conic_b, conic_c, 96'b0};
        end

        if(s3_valid)begin // three cycle latency 
            if (gaussian_cnt==0) begin
                s3_old_color_b = 16'b0;
                s3_old_color_g = 16'b0;
                s3_old_color_r = 16'b0;
                s3_old_color_t = (16'b1<<8); 
            end else begin 
                s3_old_color_b = sram_rdata_img[15:0];
                s3_old_color_g = sram_rdata_img[31:16];
                s3_old_color_r = sram_rdata_img[47:32];
                s3_old_color_t = sram_rdata_img[63:48];
            end

            /*
            if (gaussian_cnt==0) begin
                s3_old_color_b = 16'b0;
                s3_old_color_g = 16'b0;
                s3_old_color_r = 16'b0;
                s3_old_color_t = (16'b1<<8); 
            end else if(gaussian_cnt%2==0)begin 
                s3_old_color_b = sram_rdata_img[15:0];
                s3_old_color_g = sram_rdata_img[31:16];
                s3_old_color_r = sram_rdata_img[47:32];
                s3_old_color_t = sram_rdata_img[63:48];
            end else begin 
                s3_old_color_b = sram_rdata_temp[15:0];
                s3_old_color_g = sram_rdata_temp[31:16];
                s3_old_color_r = sram_rdata_temp[47:32];
                s3_old_color_t = sram_rdata_temp[63:48];
            end
            */
        end

        if (s5_valid) begin
            s5_term_a = render_product_long[9*PRODUCT_WIDTH_LONG-1:8*PRODUCT_WIDTH_LONG] >>> 8;
            s5_term_b = render_product_long[8*PRODUCT_WIDTH_LONG-1:7*PRODUCT_WIDTH_LONG] >>> 8;
            s5_term_c = render_product_long[7*PRODUCT_WIDTH_LONG-1:6*PRODUCT_WIDTH_LONG] >>> 8;
        end

        if (s7_valid) begin
            render_multiplicand[6*BIT_WIDTH-1:5*BIT_WIDTH] = color_t;
            render_multiplier[6*BIT_WIDTH-1:5*BIT_WIDTH] = exp_lut_out;
        end   

        if (s8_valid) begin
            s8_alpha_prod = render_product[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH];
            s8_alpha_rounded = (s8_alpha_prod + 32) >>> 6;
            if (!s8_p_valid) s8_alpha = 16'd0; // Use pipelined validity
            else if (s8_alpha_rounded <= 32'd1) s8_alpha = 16'd0;
            else if (s8_alpha_rounded > 32'sd253) s8_alpha = 16'd253;
            else s8_alpha = s8_alpha_rounded[15:0];
            //s8_contribution_alpha = (s8_alpha*s8_old_color_t)>>>8; // TODO : 也沒有切
        end

        if (s9_valid) begin
            render_multiplicand[5*BIT_WIDTH-1:4*BIT_WIDTH] = s9_alpha;
            render_multiplier[5*BIT_WIDTH-1:4*BIT_WIDTH] = s9_old_color_t;
        end

        if (s10_valid) begin
            s10_contribution_alpha = render_product[5*PRODUCT_WIDTH-1:4*PRODUCT_WIDTH] >>>8;
        end

        if (s11_valid) begin
            render_multiplicand[4*BIT_WIDTH-1:1*BIT_WIDTH] = {color_r, color_g, color_b};
            render_multiplier[4*BIT_WIDTH-1:1*BIT_WIDTH] = {s11_contribution_alpha, s11_contribution_alpha, s11_contribution_alpha};
        end

        if (s12_valid) begin
            render_multiplicand[1*BIT_WIDTH-1:0*BIT_WIDTH] = s12_old_color_t;
            render_multiplier[1*BIT_WIDTH-1:0*BIT_WIDTH] = s12_alpha_check;

            s12_color_final_r_q = render_product[4*PRODUCT_WIDTH-1:3*PRODUCT_WIDTH]+(s12_old_color_r <<< 8); // TODO : 也沒有切
            s12_color_final_g_q = render_product[3*PRODUCT_WIDTH-1:2*PRODUCT_WIDTH]+(s12_old_color_g <<< 8);
            s12_color_final_b_q = render_product[2*PRODUCT_WIDTH-1:1*PRODUCT_WIDTH]+(s12_old_color_b <<< 8);

        end

        if(s13_valid) begin
            // choose which sram to read and write

            /*
            if (gaussian_cnt == 0) begin
                sram_wen_img = 0;
                sram_wen_temp = 0;
            end else if (gaussian_cnt+1 ==gaussian_num | gaussian_cnt%2==1)begin // read from temp, and write to image 
                sram_wen_img = 0; 
                sram_wen_temp = 1;
            end else begin // read from image, and write to temp 
                sram_wen_img = 1;
                sram_wen_temp = 0; 
            end

            if(s13_x >= bbox_x_min & s13_x <= bbox_x_max & s13_y >= bbox_y_min & s13_y <= bbox_y_max) begin
                if (gaussian_cnt == 0) begin
                    sram_wdata_img = {color_final_t, color_final_r, color_final_g, color_final_b};
                    sram_wdata_temp = {color_final_t, color_final_r, color_final_g, color_final_b};
                end else if(gaussian_cnt+1==gaussian_num | gaussian_cnt%2==1)begin 
                    // sram_wen_img = 0;
                    sram_wdata_img = {color_final_t, color_final_r, color_final_g, color_final_b};
                end else begin 
                    // sram_wen_temp = 0;
                    sram_wdata_temp = {color_final_t, color_final_r, color_final_g, color_final_b};
                end
            end else begin 
                if (gaussian_cnt==0)begin 
                    //sram_wen_img = 0;
                    sram_wdata_img = {(16'd1<<8), 16'd0, 16'd0, 16'd0};
                    //sram_wen_temp = 0;
                    sram_wdata_temp = {(16'd1<<8), 16'd0, 16'd0, 16'd0};
                end else begin 
                    // sram_wen_img = 0;
                    sram_wdata_temp = {s13_old_color_t, s13_old_color_r, s13_old_color_g, s13_old_color_b};
                    sram_wdata_img = {s13_old_color_t, s13_old_color_r, s13_old_color_g, s13_old_color_b};
                end
            end 
            */


            sram_wen_img = 0;

            if(s13_x >= bbox_x_min & s13_x <= bbox_x_max & s13_y >= bbox_y_min & s13_y <= bbox_y_max) begin
                sram_wdata_img = {color_final_t, color_final_r, color_final_g, color_final_b};
            end else begin 
                if (gaussian_cnt==0)begin 
                    //sram_wen_img = 0;
                    sram_wdata_img = {(16'd1<<8), 16'd0, 16'd0, 16'd0};
                end else begin 
                    // sram_wen_img = 0;
                    sram_wdata_img = {s13_old_color_t, s13_old_color_r, s13_old_color_g, s13_old_color_b};
                end
            end 
        end

    end
    
    always@(*)begin 
        state_n = state; 
        read_count_n = read_count;
        current_gaussian_id = sram_rdata_sort[31:16];
        gaussian_cnt_n = gaussian_cnt;
        case(state)
            IDLE: begin
                if(enable) state_n = FETCH_SORT; 
            end
            FETCH_SORT: begin
                
                if (read_count < 3) read_count_n = read_count + 1;
                else begin
                    // Check for Invalid Gaussian (Depth = 0xFFFF)
                    if (sram_rdata_sort[15:0] == 16'hFFFF) begin
                         if (gaussian_cnt + 1 >= gaussian_num) begin
                             state_n = DONE;
                         end else begin
                             state_n = FETCH_SORT; // Or stay here to fetch next
                             gaussian_cnt_n = gaussian_cnt + 1;
                             read_count_n = 0; // Restart read delay for next item
                         end
                    end else begin
                        // Valid Gaussian
                        state_n = FETCH_PARAM;
                        read_count_n = 0;
                    end
                end
            end
            
            FETCH_PARAM: begin
                if (read_count < 3) read_count_n = read_count + 1;
                else begin
                    state_n = CALC;
                    read_count_n = 0;
                end
            end

            CALC: begin
                if (s13_done && gaussian_cnt+1 == gaussian_num ) begin
                    state_n = DONE;
                end else if(s13_done & gaussian_cnt+1 < gaussian_num)begin
                    state_n = FETCH_SORT;
                    gaussian_cnt_n = gaussian_cnt + 1;
                end else begin 
                    state_n = CALC;
                end
            end

            DONE: begin
                state_n = IDLE;
            end
        endcase
    end


    // read logic , read logic 
    always @(*)begin
        sram_raddr_input = 0;
        sram_raddr_raster = 0;
        color_b_n = color_b;
        color_g_n = color_g;
        color_r_n = color_r;
        color_t_n = color_t;
        sram_raddr_sort = gaussian_cnt;
        sram_raddr_raster = current_gaussian_id;
        sram_raddr_input = 3+ 4*current_gaussian_id;
        if(state==FETCH_PARAM & read_count==4'd3) begin
            color_b_n = sram_rdata_input[4*BIT_WIDTH-1:3*BIT_WIDTH]; 
            color_g_n = sram_rdata_input[3*BIT_WIDTH-1:2*BIT_WIDTH]; 
            color_r_n = sram_rdata_input[2*BIT_WIDTH-1:BIT_WIDTH]; 
            color_t_n = sram_rdata_input[BIT_WIDTH-1:0]; 
        end
    end

    // write logic for output image 
    // first check if the pixel is inside bbox
    // output logic 
    assign valid = state==DONE;



endmodule 