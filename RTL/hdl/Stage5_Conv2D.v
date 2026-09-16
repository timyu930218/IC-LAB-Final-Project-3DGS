module Stage5_Conv2D #(
    parameter BIT_WIDTH = 16,
    parameter LONG_BIT_WIDTH = 40,
    parameter FLOAT_WIDTH = 6,
    parameter PRODUCT_WIDTH = 32,
    parameter PRODUCT_WIDTH_LONG = 56,
    parameter RENDER_WIDTH = 128,
    parameter RENDER_HEIGHT = 128
) (
    input clk,
    input srst_n,
    input enable,
    input [6:0]gaussian_cnt, 
    input [9*PRODUCT_WIDTH-1:0] conv2d_product,
    input [9*PRODUCT_WIDTH_LONG-1:0] conv2d_product_long,
    output valid,

    // SRAM Interface (View Matrix) - 64-bit
    input  [4*BIT_WIDTH-1:0] sram_rdata_view,
    //output  [4*BIT_WIDTH-1:0] sram_wdata_view,
    output [15:0] sram_raddr_view,
    //output [15:0] sram_waddr_view,
    
    // SRAM Interface (Raster Parameter), Write Only 
    output reg [127:0] sram_wdata_raster,
    output reg [5:0] sram_waddr_raster,
    output reg [15:0] sram_wordmask_raster,
    output reg         sram_wen_raster,

    input [6*BIT_WIDTH-1:0] jacobian,
    input [9*BIT_WIDTH-1:0] conv_matrix,
    input [3*BIT_WIDTH-1:0] campos,
    output reg [4*BIT_WIDTH-1:0] conv2d,

    output reg signed [9*BIT_WIDTH-1:0] conv2d_multiplicand,
    output reg signed [9*LONG_BIT_WIDTH-1:0] conv2d_multiplicand_long,
    output reg signed [9*BIT_WIDTH-1:0] conv2d_multiplier,
    output reg signed [9*BIT_WIDTH-1:0] conv2d_multiplier_long,

    output reg [6*BIT_WIDTH-1:0] bbox_info
);

    //========================
    // DECLARE
    //========================

    localparam  IDLE = 3'b000, 
                READ = 3'b001, 
                COMPUTE = 3'b010, 
                DONE = 3'b011;
    reg [2:0] state, state_n;
    reg [4*BIT_WIDTH-1:0] conv2d_n;
    reg [3:0] read_count, read_count_n;
    reg [4:0] compute_count, compute_count_n;

    reg signed [39:0] raw_00, raw_01, raw_10, raw_11;
    reg signed [16:0] raw_00_q, raw_01_q, raw_10_q, raw_11_q;
    // datapath1 matrix declare 
    wire signed [BIT_WIDTH-1:0] conv3d_00, conv3d_01, conv3d_02, conv3d_10, conv3d_11, conv3d_12, conv3d_20, conv3d_21, conv3d_22;
    reg  signed [BIT_WIDTH-1:0] view_00,   view_01,   view_02,   view_10,   view_11,   view_12,   view_20,   view_21,   view_22;
    reg  signed [BIT_WIDTH-1:0] view_00_n, view_01_n, view_02_n, view_10_n, view_11_n, view_12_n, view_20_n, view_21_n, view_22_n;
    
    // set a temp matrix to represent the product of W @ 3Dcov
    // Increased to 40 bits to prevent overflow (sum of 3 32-bit products)
    reg signed [39:0] temp00, temp01, temp02, temp10, temp11, temp12, temp20, temp21, temp22;
    reg signed [39:0] temp00_n, temp01_n, temp02_n, temp10_n, temp11_n, temp12_n, temp20_n, temp21_n, temp22_n;

    reg signed [4*BIT_WIDTH-1:0] camera_00_q, camera_01_q, camera_02_q, camera_10_q, camera_11_q, camera_12_q, camera_20_q, camera_21_q, camera_22_q;
    reg signed [4*BIT_WIDTH-1:0] camera_00_q_n, camera_01_q_n, camera_02_q_n, camera_10_q_n, camera_11_q_n, camera_12_q_n, camera_20_q_n, camera_21_q_n, camera_22_q_n;

    // quantize camera matrix to Q_MAIN
    reg signed [BIT_WIDTH-1:0] camera_00, camera_01, camera_02, camera_10, camera_11, camera_12, camera_20, camera_21, camera_22;
    reg signed [BIT_WIDTH-1:0] camera_00_n, camera_01_n, camera_02_n, camera_10_n, camera_11_n, camera_12_n, camera_20_n, camera_21_n, camera_22_n;

    // datapath2 matrix declare 
    reg signed [BIT_WIDTH-1:0] conv2d_00, conv2d_01, conv2d_10, conv2d_11;
    reg signed [BIT_WIDTH-1:0] conv2d_00_n, conv2d_01_n, conv2d_10_n, conv2d_11_n;

    wire signed [BIT_WIDTH-1:0] jacobian_00, jacobian_01, jacobian_02, jacobian_10, jacobian_11, jacobian_12;
    // Increased to 40 bits
    reg signed [39:0] t00, t01, t02, t10, t11, t12;
    reg signed [39:0] t00_n, t01_n, t02_n, t10_n, t11_n, t12_n;

    reg signed [4*BIT_WIDTH-1:0] conv2d_00_q, conv2d_01_q, conv2d_10_q, conv2d_11_q;
    reg signed [4*BIT_WIDTH-1:0] conv2d_00_q_n, conv2d_01_q_n, conv2d_10_q_n, conv2d_11_q_n;

    // datapath3 bbox calculation 
    reg signed [6*BIT_WIDTH-1:0] bbox_info_n;
    wire signed [BIT_WIDTH-1:0] campos_x, campos_y;
    reg signed [2*BIT_WIDTH-1:0] u_q, v_q;
    reg signed [2*BIT_WIDTH-1:0] u_q_n, v_q_n;

    reg signed [BIT_WIDTH-1:0] u, v;
    reg signed [BIT_WIDTH-1:0] u_n, v_n;


    wire signed [BIT_WIDTH-1:0] rad_x, rad_y;
    wire signed [BIT_WIDTH-1:0] min_x, min_y;
    wire signed [BIT_WIDTH-1:0] max_x, max_y;

    //========================
    // DATAPATH1 (W @ 3Dcov @ Wt)
    //========================

    assign conv3d_00 = conv_matrix[9*BIT_WIDTH-1:8*BIT_WIDTH];
    assign conv3d_01 = conv_matrix[8*BIT_WIDTH-1:7*BIT_WIDTH];
    assign conv3d_02 = conv_matrix[7*BIT_WIDTH-1:6*BIT_WIDTH];
    assign conv3d_10 = conv_matrix[6*BIT_WIDTH-1:5*BIT_WIDTH];
    assign conv3d_11 = conv_matrix[5*BIT_WIDTH-1:4*BIT_WIDTH];
    assign conv3d_12 = conv_matrix[4*BIT_WIDTH-1:3*BIT_WIDTH];
    assign conv3d_20 = conv_matrix[3*BIT_WIDTH-1:2*BIT_WIDTH];
    assign conv3d_21 = conv_matrix[2*BIT_WIDTH-1:1*BIT_WIDTH];
    assign conv3d_22 = conv_matrix[1*BIT_WIDTH-1:0*BIT_WIDTH];

    // matrix multiplication W @ Sigma
    // Temp[i,j] = Row i of W . Col j of Sigma
    /*
    assign temp00 = view_00 * conv3d_00 + view_01 * conv3d_10 + view_02 * conv3d_20;
    assign temp01 = view_00 * conv3d_01 + view_01 * conv3d_11 + view_02 * conv3d_21; 
    assign temp02 = view_00 * conv3d_02 + view_01 * conv3d_12 + view_02 * conv3d_22;
    
    assign temp10 = view_10 * conv3d_00 + view_11 * conv3d_10 + view_12 * conv3d_20;
    assign temp11 = view_10 * conv3d_01 + view_11 * conv3d_11 + view_12 * conv3d_21;
    assign temp12 = view_10 * conv3d_02 + view_11 * conv3d_12 + view_12 * conv3d_22;
    
    assign temp20 = view_20 * conv3d_00 + view_21 * conv3d_10 + view_22 * conv3d_20;
    assign temp21 = view_20 * conv3d_01 + view_21 * conv3d_11 + view_22 * conv3d_21;
    assign temp22 = view_20 * conv3d_02 + view_21 * conv3d_12 + view_22 * conv3d_22;
    */

    // matrix multiplication (W@Sigma) @ W(transpose)

    /*
    assign camera_00_q = temp00 * view_00 + temp01 * view_01 + temp02 * view_02;
    assign camera_01_q = temp00 * view_10 + temp01 * view_11 + temp02 * view_12;
    assign camera_02_q = temp00 * view_20 + temp01 * view_21 + temp02 * view_22;

    assign camera_10_q = temp10 * view_00 + temp11 * view_01 + temp12 * view_02;
    assign camera_11_q = temp10 * view_10 + temp11 * view_11 + temp12 * view_12;
    assign camera_12_q = temp10 * view_20 + temp11 * view_21 + temp12 * view_22;

    assign camera_20_q = temp20 * view_00 + temp21 * view_01 + temp22 * view_02;
    assign camera_21_q = temp20 * view_10 + temp21 * view_11 + temp22 * view_12;
    assign camera_22_q = temp20 * view_20 + temp21 * view_21 + temp22 * view_22;

    */
    
    // quantize camera matrix to Q_MAIN
    /*
    assign camera_00 = (camera_00_q + (1<<11)) >>> 12;
    assign camera_01 = (camera_01_q + (1<<11)) >>> 12;
    assign camera_02 = (camera_02_q + (1<<11)) >>> 12;
    assign camera_10 = (camera_10_q + (1<<11)) >>> 12;
    assign camera_11 = (camera_11_q + (1<<11)) >>> 12;
    assign camera_12 = (camera_12_q + (1<<11)) >>> 12;
    assign camera_20 = (camera_20_q + (1<<11)) >>> 12;
    assign camera_21 = (camera_21_q + (1<<11)) >>> 12;
    assign camera_22 = (camera_22_q + (1<<11)) >>> 12;
    */

    //=======================
    // DATAPATH2(J @ Camera @ Jt)
    //=======================
    assign jacobian_00 = jacobian[6*BIT_WIDTH-1:5*BIT_WIDTH];
    assign jacobian_01 = jacobian[5*BIT_WIDTH-1:4*BIT_WIDTH];
    assign jacobian_02 = jacobian[4*BIT_WIDTH-1:3*BIT_WIDTH];
    assign jacobian_10 = jacobian[3*BIT_WIDTH-1:2*BIT_WIDTH];
    assign jacobian_11 = jacobian[2*BIT_WIDTH-1:1*BIT_WIDTH];
    assign jacobian_12 = jacobian[1*BIT_WIDTH-1:0*BIT_WIDTH];

    // T = J @ Camera (2x3 @ 3x3 -> 2x3)

    /*
    assign t00 = jacobian_00 * camera_00 + jacobian_01 * camera_10 + jacobian_02 * camera_20;
    assign t01 = jacobian_00 * camera_01 + jacobian_01 * camera_11 + jacobian_02 * camera_21;
    assign t02 = jacobian_00 * camera_02 + jacobian_01 * camera_12 + jacobian_02 * camera_22;

    assign t10 = jacobian_10 * camera_00 + jacobian_11 * camera_10 + jacobian_12 * camera_20;
    assign t11 = jacobian_10 * camera_01 + jacobian_11 * camera_11 + jacobian_12 * camera_21;
    assign t12 = jacobian_10 * camera_02 + jacobian_11 * camera_12 + jacobian_12 * camera_22;
    */
    // Cov2D = T @ J.T (2x3 @ 3x2 -> 2x2)
    // J.T = [[j00, j10], [j01, j11], [j02, j12]]
    /*
    assign conv2d_00_q = t00 * jacobian_00 + t01 * jacobian_01 + t02 * jacobian_02;
    assign conv2d_01_q = t00 * jacobian_10 + t01 * jacobian_11 + t02 * jacobian_12;

    assign conv2d_10_q = t10 * jacobian_00 + t11 * jacobian_01 + t12 * jacobian_02;
    assign conv2d_11_q = t10 * jacobian_10 + t11 * jacobian_11 + t12 * jacobian_12;
    */

    // quantize to Qmain 
    // Add Low Pass Filter (+0.3) to diagonal elements
    // 0.3 in Q10.6 is round(0.3 * 64) = 19
    /*
    assign conv2d_00 = ((conv2d_00_q + (1<<11)) >>> 12) + 19;
    assign conv2d_01 = (conv2d_01_q + (1<<11)) >>> 12;
    assign conv2d_10 = (conv2d_10_q + (1<<11)) >>> 12;
    assign conv2d_11 = ((conv2d_11_q + (1<<11)) >>> 12) + 19;
    */

    //=======================
    // DATAPATH3(BBox)
    //=======================

    // screen position = x * j00 + (width/2)
    // screen position = y * (-j11) + (height/2)

    
    assign campos_x = campos[3*BIT_WIDTH-1:2*BIT_WIDTH];
    assign campos_y = campos[2*BIT_WIDTH-1:BIT_WIDTH];

    /*
    assign u_q = campos_x * jacobian_00 + ((RENDER_WIDTH/2)<<12);
    assign v_q = (campos_y * jacobian_11) + ((RENDER_HEIGHT/2)<<12);
    assign u = (u_q ) >>> 6;
    assign v = (v_q ) >>> 6;

    */

    // lut find square root 
    // lut find square root 
    // Logic: 
    // IF RENDER_WIDTH=128 (4x Cov2D values):
    //   Input: Shift right by 14 (div 4 vs standard div 1). 
    //          Standard is >> 12. New is >> 14. Delta is >> 2 (div 4).
    //   Output: Sqrt(x/4) = Sqrt(x)/2. Multiply by 2 (<< 1) to recover.
    // IF RENDER_WIDTH=64:
    //   Input: Shift right by 12.
    //   Output: Direct.
    
    wire signed [15:0] sqrt_in_x;
    wire signed [15:0] sqrt_in_y;
    wire [15:0] lut_out_x;
    wire [15:0] lut_out_y;

    // Use WIDE 32-bit variables for saturation check before casting to 16-bit input
    wire signed [31:0] sqrt_in_x_raw;
    wire signed [31:0] sqrt_in_y_raw;
    wire signed [15:0] sat_limit;

    // Clamp high-precision accumulator to 16-bit Q10.6 equivalent range (shifted by 12)
    // 32767 << 12 = 134213632. 
    // This prevents the unclamped accumulator from exceeding the max representable Q10.6 value
    // which matches the Golden model's behavior.
    
    wire signed [63:0] conv2d_00_clamped;
    wire signed [63:0] conv2d_11_clamped;
    wire signed [63:0] clamp_val = 64'd134213632; 

    assign conv2d_00_clamped = (conv2d_00_q > clamp_val) ? clamp_val : 
                               (conv2d_00_q < -clamp_val) ? -clamp_val : conv2d_00_q;

    assign conv2d_11_clamped = (conv2d_11_q > clamp_val) ? clamp_val : 
                               (conv2d_11_q < -clamp_val) ? -clamp_val : conv2d_11_q;

    assign sqrt_in_x_raw = (RENDER_WIDTH >= 128) ? 
                       ((conv2d_00_clamped + (1<<11)) >>> 14) + (19 >>> 2) : 
                       ((conv2d_00_clamped + (1<<11)) >>> 12) + 19;
                       
    assign sqrt_in_y_raw = (RENDER_WIDTH >= 128) ? 
                       ((conv2d_11_clamped + (1<<11)) >>> 14) + (19 >>> 2) : 
                       ((conv2d_11_clamped + (1<<11)) >>> 12) + 19;

    // Saturation limit depends on scale.
    // In 64 mode: Limit is 32767 (Max 16-bit signed).
    // In 128 mode: Input is effectively (Cov / 4).
    // To match Python's 16-bit clamp on Cov, we must clamp Input to (32767 / 4) = 8191.
    assign sat_limit = 16'd32767;

    assign sqrt_in_x = (sqrt_in_x_raw > {16'b0, sat_limit}) ? sat_limit : 
                       (sqrt_in_x_raw < 0) ? 16'd0 : sqrt_in_x_raw[15:0];

    assign sqrt_in_y = (sqrt_in_y_raw > {16'b0, sat_limit}) ? sat_limit : 
                       (sqrt_in_y_raw < 0) ? 16'd0 : sqrt_in_y_raw[15:0];

    sqrt_lut sqrt_inst (
        .clk(clk),
        .in_data(sqrt_in_x),
        .out(lut_out_x)
    );
    sqrt_lut sqrt_inst2 (
        .clk(clk),
        .in_data(sqrt_in_y),
        .out(lut_out_y)
    );

    // If 128 mode, output was Sqrt(X/4) = Res/2. Multiply by 2.
    assign rad_x = (RENDER_WIDTH >= 128) ? (lut_out_x << 1) : lut_out_x;
    assign rad_y = (RENDER_WIDTH >= 128) ? (lut_out_y << 1) : lut_out_y;

    // quantize to integer
    assign min_x = ((u-rad_x)<0) ? 0 : (u - rad_x) >>> 6;
    assign min_y = ((v-rad_y)<0) ? 0 : (v - rad_y) >>> 6;

    wire signed [BIT_WIDTH-1:0] max_x_q, max_y_q;
    assign max_x_q = u + rad_x;
    assign max_y_q = v + rad_y;
    
    assign max_x = ((max_x_q >>> 6) >= RENDER_WIDTH) ? (RENDER_WIDTH-1) : (max_x_q >>> 6);
    assign max_y = ((max_y_q >>> 6) >= RENDER_HEIGHT)? (RENDER_HEIGHT-1): (max_y_q >>> 6);

    //========================
    // FSM
    //========================

    always@(posedge clk)begin 
        if(!srst_n)begin 
            state <= IDLE;
            read_count <= 4'b0000;
            compute_count <= 4'd0;
            conv2d <= 0;
            view_00 <= 0; view_01 <= 0; view_02 <= 0;
            view_10 <= 0; view_11 <= 0; view_12 <= 0;
            view_20 <= 0; view_21 <= 0; view_22 <= 0;
            bbox_info <= 0;
        end else begin 
            state <= state_n;
            read_count <= read_count_n;
            compute_count <= compute_count_n;
            conv2d <= conv2d_n;
            view_00 <= view_00_n; view_01 <= view_01_n; view_02 <= view_02_n;
            view_10 <= view_10_n; view_11 <= view_11_n; view_12 <= view_12_n;
            view_20 <= view_20_n; view_21 <= view_21_n; view_22 <= view_22_n;

            temp00 <= temp00_n; temp01 <= temp01_n; temp02 <= temp02_n;
            temp10 <= temp10_n; temp11 <= temp11_n; temp12 <= temp12_n;
            temp20 <= temp20_n; temp21 <= temp21_n; temp22 <= temp22_n;

            camera_00_q <= camera_00_q_n; camera_01_q <= camera_01_q_n; camera_02_q <= camera_02_q_n;
            camera_10_q <= camera_10_q_n; camera_11_q <= camera_11_q_n; camera_12_q <= camera_12_q_n;
            camera_20_q <= camera_20_q_n; camera_21_q <= camera_21_q_n; camera_22_q <= camera_22_q_n;

            camera_00 <= camera_00_n; camera_01 <= camera_01_n; camera_02 <= camera_02_n;
            camera_10 <= camera_10_n; camera_11 <= camera_11_n; camera_12 <= camera_12_n;
            camera_20 <= camera_20_n; camera_21 <= camera_21_n; camera_22 <= camera_22_n;

            t00 <= t00_n; t01 <= t01_n; t02 <= t02_n;
            t10 <= t10_n; t11 <= t11_n; t12 <= t12_n;

            conv2d_00_q <= conv2d_00_q_n; conv2d_01_q <= conv2d_01_q_n;
            conv2d_10_q <= conv2d_10_q_n; conv2d_11_q <= conv2d_11_q_n;

            conv2d_00 <= conv2d_00_n; conv2d_01 <= conv2d_01_n; 
            conv2d_10 <= conv2d_10_n; conv2d_11 <= conv2d_11_n;

            bbox_info <= bbox_info_n;

            u_q <= u_q_n;
            v_q <= v_q_n;

            u <= u_n;
            v <= v_n;

        end
    end

    always@* begin
        conv2d_multiplicand = 0;
        conv2d_multiplicand_long = 0;
        conv2d_multiplier = 0;
        conv2d_multiplier_long = 0;
        case (compute_count)
            0: begin
                conv2d_multiplicand = {view_00, view_01, view_02, view_00, view_01, view_02, view_00, view_01, view_02};
                conv2d_multiplier = {conv3d_00, conv3d_10, conv3d_20, conv3d_01, conv3d_11, conv3d_21, conv3d_02, conv3d_12, conv3d_22};
            end

            1: begin
                conv2d_multiplicand = {view_10, view_11, view_12, view_10, view_11, view_12, view_10, view_11, view_12};
                conv2d_multiplier = {conv3d_00, conv3d_10, conv3d_20, conv3d_01, conv3d_11, conv3d_21, conv3d_02, conv3d_12, conv3d_22};
            end

            2: begin
                conv2d_multiplicand = {view_20, view_21, view_22, view_20, view_21, view_22, view_20, view_21, view_22};
                conv2d_multiplier = {conv3d_00, conv3d_10, conv3d_20, conv3d_01, conv3d_11, conv3d_21, conv3d_02, conv3d_12, conv3d_22};

                conv2d_multiplicand_long = {temp00, temp01, temp02, temp00, temp01, temp02, temp00, temp01, temp02};
                conv2d_multiplier_long = {view_00, view_01, view_02, view_10, view_11, view_12, view_20, view_21, view_22};
            end

            3: begin
                conv2d_multiplicand_long = {temp10, temp11, temp12, temp10, temp11, temp12, temp10, temp11, temp12};
                conv2d_multiplier_long = {view_00, view_01, view_02, view_10, view_11, view_12, view_20, view_21, view_22};
            end

            4: begin
                conv2d_multiplicand_long = {temp20, temp21, temp22, temp20, temp21, temp22, temp20, temp21, temp22};
                conv2d_multiplier_long = {view_00, view_01, view_02, view_10, view_11, view_12, view_20, view_21, view_22};
            end

            9: begin
                conv2d_multiplicand = {jacobian_00, jacobian_01, jacobian_02, jacobian_00, jacobian_01, jacobian_02, jacobian_00, jacobian_01, jacobian_02};
                conv2d_multiplier = {camera_00, camera_10, camera_20, camera_01, camera_11, camera_21, camera_02, camera_12, camera_22};
            end

            10: begin
                conv2d_multiplicand = {jacobian_10, jacobian_11, jacobian_12, jacobian_10, jacobian_11, jacobian_12, jacobian_10, jacobian_11, jacobian_12};
                conv2d_multiplier = {camera_00, camera_10, camera_20, camera_01, camera_11, camera_21, camera_02, camera_12, camera_22};
            end

            11: begin
                conv2d_multiplicand_long = {t00, t01, t02, t00, t01, t02, 120'b0};
                conv2d_multiplier_long = {jacobian_00, jacobian_01, jacobian_02, jacobian_10, jacobian_11, jacobian_12, 48'b0};
            end

            12: begin
            
                conv2d_multiplicand_long = {t10, t11, t12, t10, t11, t12, 120'b0};
                conv2d_multiplier_long = {jacobian_00, jacobian_01, jacobian_02, jacobian_10, jacobian_11, jacobian_12, 48'b0};

            end

            14: begin
                conv2d_multiplicand = {campos_x, campos_y, 112'b0};
                conv2d_multiplier = {jacobian_00, jacobian_11, 112'b0};
            end

            default:;
        endcase
    end


    always@(*)begin 
        state_n = state;
        read_count_n = read_count;
        compute_count_n = compute_count;
        conv2d_n = conv2d;

        temp00_n = temp00; temp01_n = temp01; temp02_n = temp02;
        temp10_n = temp10; temp11_n = temp11; temp12_n = temp12;
        temp20_n = temp20; temp21_n = temp21; temp22_n = temp22;

        camera_00_q_n = camera_00_q; camera_01_q_n = camera_01_q; camera_02_q_n = camera_02_q;
        camera_10_q_n = camera_10_q; camera_11_q_n = camera_11_q; camera_12_q_n = camera_12_q;
        camera_20_q_n = camera_20_q; camera_21_q_n = camera_21_q; camera_22_q_n = camera_22_q;

        camera_00_n = camera_00; camera_01_n = camera_01; camera_02_n = camera_02;
        camera_10_n = camera_10; camera_11_n = camera_11; camera_12_n = camera_12;
        camera_20_n = camera_20; camera_21_n = camera_21; camera_22_n = camera_22;

        t00_n = t00; t01_n = t01; t02_n = t02;
        t10_n = t10; t11_n = t11; t12_n = t12;

        conv2d_00_q_n = conv2d_00_q; conv2d_01_q_n = conv2d_01_q;
        conv2d_10_q_n = conv2d_10_q; conv2d_11_q_n = conv2d_11_q;

        conv2d_00_n = conv2d_00; conv2d_01_n = conv2d_01; 
        conv2d_10_n = conv2d_11; conv2d_11_n = conv2d_11;

        bbox_info_n = bbox_info;

        u_q_n = u_q;
        v_q_n = v_q;

        u_n = u;
        v_n = v;
        
        sram_waddr_raster = 0;
        sram_wdata_raster = 0;
        sram_wordmask_raster = 16'b0;
        sram_wen_raster = 1;

        case(state)
            IDLE: begin
                read_count_n = 0; 
                if(enable) begin
                    state_n = READ;
                end
            end
            READ: begin
                state_n = READ;
                read_count_n = read_count + 1;
                if(read_count==5) state_n = COMPUTE; // 三個cycle的latency(檔input, output flipflop) , 加上送資料的兩個cycle
            end
            COMPUTE: begin
                case (compute_count)

                    0: begin
                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    1: begin
                        temp00_n = $signed(conv2d_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH]);

                        temp01_n = $signed(conv2d_product[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[5*PRODUCT_WIDTH-1:4*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[4*PRODUCT_WIDTH-1:3*PRODUCT_WIDTH]);

                        temp02_n = $signed(conv2d_product[3*PRODUCT_WIDTH-1:2*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[2*PRODUCT_WIDTH-1:1*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[1*PRODUCT_WIDTH-1:0*PRODUCT_WIDTH]);

                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    2: begin
                        temp10_n = $signed(conv2d_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH]);

                        temp11_n = $signed(conv2d_product[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[5*PRODUCT_WIDTH-1:4*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[4*PRODUCT_WIDTH-1:3*PRODUCT_WIDTH]);

                        temp12_n = $signed(conv2d_product[3*PRODUCT_WIDTH-1:2*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[2*PRODUCT_WIDTH-1:1*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[1*PRODUCT_WIDTH-1:0*PRODUCT_WIDTH]);

                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    3: begin
                        temp20_n = $signed(conv2d_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH]);

                        temp21_n = $signed(conv2d_product[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[5*PRODUCT_WIDTH-1:4*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[4*PRODUCT_WIDTH-1:3*PRODUCT_WIDTH]);

                        temp22_n = $signed(conv2d_product[3*PRODUCT_WIDTH-1:2*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[2*PRODUCT_WIDTH-1:1*PRODUCT_WIDTH]) + 
                                    $signed(conv2d_product[1*PRODUCT_WIDTH-1:0*PRODUCT_WIDTH]);

                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    4: begin

                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    5: begin
                        camera_00_q_n = $signed(conv2d_product_long[9*PRODUCT_WIDTH_LONG-1:8*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[8*PRODUCT_WIDTH_LONG-1:7*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[7*PRODUCT_WIDTH_LONG-1:6*PRODUCT_WIDTH_LONG]);

                        camera_01_q_n = $signed(conv2d_product_long[6*PRODUCT_WIDTH_LONG-1:5*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[5*PRODUCT_WIDTH_LONG-1:4*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[4*PRODUCT_WIDTH_LONG-1:3*PRODUCT_WIDTH_LONG]);

                        camera_02_q_n = $signed(conv2d_product_long[3*PRODUCT_WIDTH_LONG-1:2*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[2*PRODUCT_WIDTH_LONG-1:1*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[1*PRODUCT_WIDTH_LONG-1:0*PRODUCT_WIDTH_LONG]);
                        
                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    6: begin
                        camera_10_q_n = $signed(conv2d_product_long[9*PRODUCT_WIDTH_LONG-1:8*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[8*PRODUCT_WIDTH_LONG-1:7*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[7*PRODUCT_WIDTH_LONG-1:6*PRODUCT_WIDTH_LONG]);

                        camera_11_q_n = $signed(conv2d_product_long[6*PRODUCT_WIDTH_LONG-1:5*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[5*PRODUCT_WIDTH_LONG-1:4*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[4*PRODUCT_WIDTH_LONG-1:3*PRODUCT_WIDTH_LONG]);

                        camera_12_q_n = $signed(conv2d_product_long[3*PRODUCT_WIDTH_LONG-1:2*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[2*PRODUCT_WIDTH_LONG-1:1*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[1*PRODUCT_WIDTH_LONG-1:0*PRODUCT_WIDTH_LONG]);
                    
                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    7: begin
                        camera_20_q_n = $signed(conv2d_product_long[9*PRODUCT_WIDTH_LONG-1:8*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[8*PRODUCT_WIDTH_LONG-1:7*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[7*PRODUCT_WIDTH_LONG-1:6*PRODUCT_WIDTH_LONG]);

                        camera_21_q_n = $signed(conv2d_product_long[6*PRODUCT_WIDTH_LONG-1:5*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[5*PRODUCT_WIDTH_LONG-1:4*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[4*PRODUCT_WIDTH_LONG-1:3*PRODUCT_WIDTH_LONG]);

                        camera_22_q_n = $signed(conv2d_product_long[3*PRODUCT_WIDTH_LONG-1:2*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[2*PRODUCT_WIDTH_LONG-1:1*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[1*PRODUCT_WIDTH_LONG-1:0*PRODUCT_WIDTH_LONG]);
                    
                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    8: begin
                        camera_00_n = (camera_00_q + (1<<11)) >>> 12;
                        camera_01_n = (camera_01_q + (1<<11)) >>> 12;
                        camera_02_n = (camera_02_q + (1<<11)) >>> 12;
                        camera_10_n = (camera_10_q + (1<<11)) >>> 12;
                        camera_11_n = (camera_11_q + (1<<11)) >>> 12;
                        camera_12_n = (camera_12_q + (1<<11)) >>> 12;
                        camera_20_n = (camera_20_q + (1<<11)) >>> 12;
                        camera_21_n = (camera_21_q + (1<<11)) >>> 12;
                        camera_22_n = (camera_22_q + (1<<11)) >>> 12;

                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    9: begin
                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    10: begin
                        t00_n = $signed(conv2d_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH]) + 
                                $signed(conv2d_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH]) + 
                                $signed(conv2d_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH]);

                        t01_n = $signed(conv2d_product[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH]) + 
                                $signed(conv2d_product[5*PRODUCT_WIDTH-1:4*PRODUCT_WIDTH]) + 
                                $signed(conv2d_product[4*PRODUCT_WIDTH-1:3*PRODUCT_WIDTH]);

                        t02_n = $signed(conv2d_product[3*PRODUCT_WIDTH-1:2*PRODUCT_WIDTH]) + 
                                $signed(conv2d_product[2*PRODUCT_WIDTH-1:1*PRODUCT_WIDTH]) + 
                                $signed(conv2d_product[1*PRODUCT_WIDTH-1:0*PRODUCT_WIDTH]);            
                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    11: begin
                        t10_n = $signed(conv2d_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH]) + 
                                $signed(conv2d_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH]) + 
                                $signed(conv2d_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH]);

                        t11_n = $signed(conv2d_product[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH]) + 
                                $signed(conv2d_product[5*PRODUCT_WIDTH-1:4*PRODUCT_WIDTH]) + 
                                $signed(conv2d_product[4*PRODUCT_WIDTH-1:3*PRODUCT_WIDTH]);

                        t12_n = $signed(conv2d_product[3*PRODUCT_WIDTH-1:2*PRODUCT_WIDTH]) + 
                                $signed(conv2d_product[2*PRODUCT_WIDTH-1:1*PRODUCT_WIDTH]) + 
                                $signed(conv2d_product[1*PRODUCT_WIDTH-1:0*PRODUCT_WIDTH]);            
                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    12: begin
                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    13: begin

                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    14: begin
                        conv2d_00_q_n = $signed(conv2d_product_long[9*PRODUCT_WIDTH_LONG-1:8*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[8*PRODUCT_WIDTH_LONG-1:7*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[7*PRODUCT_WIDTH_LONG-1:6*PRODUCT_WIDTH_LONG]);

                        conv2d_01_q_n = $signed(conv2d_product_long[6*PRODUCT_WIDTH_LONG-1:5*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[5*PRODUCT_WIDTH_LONG-1:4*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[4*PRODUCT_WIDTH_LONG-1:3*PRODUCT_WIDTH_LONG]);
                    
                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    15: begin
                        conv2d_10_q_n = $signed(conv2d_product_long[9*PRODUCT_WIDTH_LONG-1:8*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[8*PRODUCT_WIDTH_LONG-1:7*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[7*PRODUCT_WIDTH_LONG-1:6*PRODUCT_WIDTH_LONG]);

                        conv2d_11_q_n = $signed(conv2d_product_long[6*PRODUCT_WIDTH_LONG-1:5*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[5*PRODUCT_WIDTH_LONG-1:4*PRODUCT_WIDTH_LONG]) + 
                                        $signed(conv2d_product_long[4*PRODUCT_WIDTH_LONG-1:3*PRODUCT_WIDTH_LONG]);
                    
                        u_q_n = $signed(conv2d_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH]) + ((RENDER_WIDTH/2)<<12);
                        v_q_n = $signed(conv2d_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH]) + ((RENDER_HEIGHT/2)<<12);

                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    16: begin
                            // Use WIDE 40-bit variables for saturation check
                            raw_00 = ((conv2d_00_q + (1<<11)) >>> 12) + 19;
                            raw_01 = (conv2d_01_q + (1<<11)) >>> 12;
                            raw_10 = (conv2d_10_q + (1<<11)) >>> 12;
                            raw_11 = ((conv2d_11_q + (1<<11)) >>> 12) + 19;

                            // Clamping Logic 
                            // 00
                            if (raw_00 > 32767) conv2d_00_n = 32767;
                            else if (raw_00 < -32768) conv2d_00_n = -32768;
                            else conv2d_00_n = raw_00[15:0];

                            // 01
                            if (raw_01 > 32767) conv2d_01_n = 32767;
                            else if (raw_01 < -32768) conv2d_01_n = -32768;
                            else conv2d_01_n = raw_01[15:0];

                            // 10
                            if (raw_10 > 32767) conv2d_10_n = 32767;
                            else if (raw_10 < -32768) conv2d_10_n = -32768;
                            else conv2d_10_n = raw_10[15:0];

                            // 11
                            if (raw_11 > 32767) conv2d_11_n = 32767;
                            else if (raw_11 < -32768) conv2d_11_n = -32768;
                            else conv2d_11_n = raw_11[15:0];

                        u_n = (u_q ) >>> 6;
                        v_n = (v_q ) >>> 6;

                        conv2d_n = {conv2d_00_n, conv2d_01_n, conv2d_10_n, conv2d_11_n};

                        state_n = COMPUTE;
                        compute_count_n = compute_count + 1;
                    end

                    17: begin
                        state_n = DONE;
                        compute_count_n = 0;
                        bbox_info_n = {u, v, min_x, max_x, min_y, max_y};
                        sram_waddr_raster = gaussian_cnt;
                        sram_wdata_raster = {16'b0 , rad_y, rad_x, v, u, 48'b0};
                        sram_wordmask_raster = { 10'b0, 6'b111111 };
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

    // set address logic 
    // 前四個cycle分別送出addr 0, 1, 2
    assign sram_raddr_view = read_count;
    // read view matrix, three cycle latency, 所以用read_count-3
    always@(*)begin 
        view_00_n = view_00; view_01_n = view_01; view_02_n = view_02;
        view_10_n = view_10; view_11_n = view_11; view_12_n = view_12;
        view_20_n = view_20; view_21_n = view_21; view_22_n = view_22;

        case(read_count)
            3: begin
                // Corrected bit slicing: LSB is Elem 0, MSB is Elem 3
                // Row 0: view[0], view[1], view[2]
                view_00_n = sram_rdata_view[BIT_WIDTH-1:0];             // view[0]
                view_01_n = sram_rdata_view[2*BIT_WIDTH-1:BIT_WIDTH];   // view[1]
                view_02_n = sram_rdata_view[3*BIT_WIDTH-1:2*BIT_WIDTH]; // view[2]
            end
            4: begin
                // Row 1: view[4], view[5], view[6]
                view_10_n = sram_rdata_view[BIT_WIDTH-1:0];             // view[4]
                view_11_n = sram_rdata_view[2*BIT_WIDTH-1:BIT_WIDTH];   // view[5]
                view_12_n = sram_rdata_view[3*BIT_WIDTH-1:2*BIT_WIDTH]; // view[6]
            end
            5: begin
                // Row 2: view[8], view[9], view[10]
                view_20_n = sram_rdata_view[BIT_WIDTH-1:0];             // view[8]
                view_21_n = sram_rdata_view[2*BIT_WIDTH-1:BIT_WIDTH];   // view[9]
                view_22_n = sram_rdata_view[3*BIT_WIDTH-1:2*BIT_WIDTH]; // view[10]
            end
            default: begin end
        endcase
    end
    
    // output 
    assign valid = (state==DONE);

endmodule