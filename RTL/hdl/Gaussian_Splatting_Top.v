module Gaussian_Splatting_Top #(
    parameter BIT_WIDTH = 16, 
    parameter NUM_GAUSSIANS = 64,
    parameter RENDER_WIDTH = 128,
    parameter RENDER_HEIGHT = 128,
    parameter RENDER_WIDTH_LOG = 7
)(
    input clk,
    input srst_n,
    input enable,
    output reg valid,
    input [6:0] gaussian_num,
    
    // SRAM Interface (Input Data) - 64-bit
    input  [63:0] sram_rdata_input,
    output reg [63:0] sram_wdata_input,
    output reg [15:0]  sram_raddr_input,
    output reg [15:0]  sram_waddr_input,
    output reg         sram_wen_input,

    // SRAM Interface (View Matrix) - 64-bit
    input  [63:0] sram_rdata_view,
    output reg [63:0] sram_wdata_view,
    output reg [15:0] sram_raddr_view,
    output reg [15:0] sram_waddr_view,
    output reg         sram_wen_view,

    // SRAM Interface (Sort List) - 48-bit
    input  [31:0] sram_rdata_sort,
    output reg [31:0] sram_wdata_sort,
    output reg [5:0] sram_raddr_sort,
    output reg [5:0] sram_waddr_sort,
    output reg         sram_wen_sort,

    // SRAM Interface (Output Image) - 48-bit
    input  [63:0] sram_rdata_img,
    output reg [63:0] sram_wdata_img,
    output reg [15:0] sram_raddr_img,
    output reg [15:0] sram_waddr_img,
    output reg         sram_wen_img, 

    // SRAM Interface (Output Temp) - 64-bit
    //input  [63:0] sram_rdata_temp,
    //output reg [63:0] sram_wdata_temp,
    //output reg [15:0] sram_raddr_temp,
    //output reg [15:0] sram_waddr_temp,
    //output reg         sram_wen_temp,

    // SRAM Interface (Raster Parameter) - 128-bit
    input  [127:0] sram_rdata_raster,
    output reg [127:0] sram_wdata_raster,
    output reg [5:0] sram_raddr_raster,
    output reg [5:0] sram_waddr_raster,
    output reg [15:0] sram_wordmask_raster,
    output reg         sram_wen_raster
);


    //========================
    // PARAMETER
    //========================
    localparam N_MULT = 9;
    localparam PRODUCT_WIDTH = 32;
    localparam LONG_BIT_WIDTH = 40;
    localparam PRODUCT_WIDTH_LONG = 56;

    //========================
    // DECLARE
    //========================

    // control unit 
    wire [3:0] state;
    wire [6:0] gaussian_cnt;
    wire [6:0] valid_gaussian_num; 
    // rot module 
    wire rot_valid;
    wire rot_enable; 
    wire [4*BIT_WIDTH-1:0] rot_sram_wdata_input;
    wire [15:0] rot_sram_raddr_input;
    wire [15:0] rot_sram_waddr_input;
    wire [9*BIT_WIDTH-1:0] rotation_matrix;
    // 3dcov module 
    wire conv_valid;
    wire conv_enable; 
    wire [4*BIT_WIDTH-1:0] conv_sram_wdata_input;
    wire [15:0] conv_sram_raddr_input;
    wire [15:0] conv_sram_waddr_input;
    wire [9*BIT_WIDTH-1:0] conv_matrix;
    // campos module 
    wire campos_valid;
    wire campos_enable; 
    wire [4*BIT_WIDTH-1:0] campos_sram_wdata_input;
    wire [15:0] campos_sram_raddr_input;
    wire [15:0] campos_sram_waddr_input;
    wire [4*BIT_WIDTH-1:0] campos_sram_wdata_view;
    wire [15:0] campos_sram_raddr_view;
    wire [15:0] campos_sram_waddr_view;
    wire [3*BIT_WIDTH-1:0] campos;
    wire [31:0] campos_sram_wdata_sort;
    //wire [5:0] campos_sram_raddr_sort;
    wire [5:0] campos_sram_waddr_sort;
    wire campos_sram_wen_sort;

    // jacobian module 
    wire jacobian_valid;
    wire jacobian_enable; 
    wire [6*BIT_WIDTH-1:0] jacobian;

    // conv2d module 
    wire conv2d_valid;
    wire conv2d_enable; 
    wire [4*BIT_WIDTH-1:0] conv2d;
    wire [4*BIT_WIDTH-1:0] conv2d_sram_wdata_view;
    wire [15:0] conv2d_sram_raddr_view;
    wire [15:0] conv2d_sram_waddr_view;
    wire [6*BIT_WIDTH-1:0] bbox_info;
    wire [5:0] conv2d_sram_waddr_raster;
    wire [127:0] conv2d_sram_wdata_raster;
    wire [15:0] conv2d_sram_wordmask_raster;
    wire conv2d_sram_wen_raster;

    // conic module
    wire conic_valid;
    wire conic_enable;
    wire [3*BIT_WIDTH-1:0] conic_coeff;
    wire [5:0] conic_sram_waddr_raster;
    wire [127:0] conic_sram_wdata_raster;
    wire [15:0] conic_sram_wordmask_raster;
    wire conic_sram_wen_raster;

    // sort module 
    wire sort_valid;
    wire sort_enable;
    wire [2*BIT_WIDTH-1:0] sort_list;
    wire [31:0] sort_sram_wdata_sort;
    wire [5:0] sort_sram_raddr_sort;
    wire [5:0] sort_sram_waddr_sort;
    wire sort_sram_wen_sort;

    // render module 
    wire render_valid;
    wire render_enable;
    wire [63:0] render_sram_wdata_input;
    wire [15:0] render_sram_raddr_input;
    wire [15:0] render_sram_waddr_input; 
    wire [63:0] render_sram_wdata_img;
    wire [15:0] render_sram_raddr_img;
    wire [15:0] render_sram_waddr_img;
    wire render_sram_wen_img;
    //wire [63:0] render_sram_wdata_temp;
    //wire [15:0] render_sram_raddr_temp;
    //wire [15:0] render_sram_waddr_temp;
    //wire render_sram_wen_temp;
    wire [5:0] render_sram_raddr_raster;
    wire [5:0] render_sram_raddr_sort;


    // input flip flop declare 
    reg enable_reg;
    reg  [63:0] sram_rdata_input_reg;
    reg  [63:0] sram_rdata_view_reg;
    reg  [63:0] sram_rdata_img_reg;
    reg  [31:0] sram_rdata_sort_reg;
    reg  [127:0] sram_rdata_raster_reg;
    //reg  [63:0] sram_rdata_temp_reg;
    // output flip flop declare
    wire valid_n;
    reg [63:0]  sram_wdata_input_n; 
    reg [15:0]  sram_raddr_input_n; 
    reg [15:0]  sram_waddr_input_n; 
    reg         sram_wen_input_n; 
    reg [63:0]  sram_wdata_view_n; 
    reg [15:0]  sram_raddr_view_n; 
    reg [15:0]  sram_waddr_view_n; 
    reg         sram_wen_view_n; 
    reg [63:0]  sram_wdata_img_n; 
    reg [15:0]  sram_raddr_img_n; 
    reg [15:0]  sram_waddr_img_n; 
    reg         sram_wen_img_n; 
    //reg [63:0]  sram_wdata_temp_n;
    //reg [15:0]  sram_raddr_temp_n;
    //reg [15:0]  sram_waddr_temp_n;
    //reg         sram_wen_temp_n;
    reg [31:0]  sram_wdata_sort_n; 
    reg [15:0]  sram_raddr_sort_n; 
    reg [15:0]  sram_waddr_sort_n; 
    reg         sram_wen_sort_n; 
    reg [127:0] sram_wdata_raster_n;
    reg [15:0] sram_wordmask_raster_n;
    reg [5:0] sram_raddr_raster_n;
    reg [5:0] sram_waddr_raster_n;
    reg sram_wen_raster_n;


    ///multiplication use
    reg signed [9*BIT_WIDTH-1:0] multiplicand_0;
    reg signed [9*LONG_BIT_WIDTH-1:0] multiplicand_long_0;
    reg signed [9*BIT_WIDTH-1:0] multiplier_0;
    reg signed [9*BIT_WIDTH-1:0] multiplier_long_0;

    wire signed [9*PRODUCT_WIDTH-1:0] product_0;
    wire signed [9*PRODUCT_WIDTH_LONG-1:0] product_long_0;

    wire signed [9*BIT_WIDTH-1:0] rot_multiplicand;
    wire signed [9*BIT_WIDTH-1:0] rot_multiplier;
    wire signed [9*BIT_WIDTH-1:0] conv_multiplicand;
    wire signed [9*BIT_WIDTH-1:0] conv_multiplier;
    wire signed [9*BIT_WIDTH-1:0] campos_multiplicand;
    wire signed [9*BIT_WIDTH-1:0] campos_multiplier;
    wire signed [9*BIT_WIDTH-1:0] jacobian_multiplicand;
    wire signed [9*BIT_WIDTH-1:0] jacobian_multiplier;
    wire signed [9*BIT_WIDTH-1:0] conv2d_multiplicand;
    wire signed [9*LONG_BIT_WIDTH-1:0] conv2d_multiplicand_long;
    wire signed [9*BIT_WIDTH-1:0] conv2d_multiplier;
    wire signed [9*BIT_WIDTH-1:0] conv2d_multiplier_long;
    wire signed [9*BIT_WIDTH-1:0] conic_multiplicand;
    wire signed [9*BIT_WIDTH-1:0] conic_multiplier;
    wire signed [9*LONG_BIT_WIDTH-1:0] conic_multiplicand_long;
    wire signed [9*BIT_WIDTH-1:0] conic_multiplier_long;
    wire signed [9*BIT_WIDTH-1:0] render_multiplicand;
    wire signed [9*BIT_WIDTH-1:0] render_multiplier;
    wire signed [9*LONG_BIT_WIDTH-1:0] render_multiplicand_long;
    wire signed [9*BIT_WIDTH-1:0] render_multiplier_long;

    ///division use

    reg [35:0] dividend;
    reg [35:0] divisor;
    wire [35:0] quotient;
    wire divide_by_0;

    wire [35:0] jacobian_dividend;
    wire [35:0] jacobian_divisor;

    wire [35:0] conic_dividend;
    wire [35:0] conic_divisor;

    // integer 
    integer i; 

    //========================
    // TESTBENCH
    //========================
    // testbench reference 

    /*
    reg [9*BIT_WIDTH-1:0] debug_stage1_data[0:NUM_GAUSSIANS-1]; 
    reg [9*BIT_WIDTH-1:0] debug_stage1_data_n[0:NUM_GAUSSIANS-1]; 
    // assign debug_stage1_data = rotation_matrix; 
    reg [9*BIT_WIDTH-1:0] debug_stage2_data[0:NUM_GAUSSIANS-1]; 
    reg [9*BIT_WIDTH-1:0] debug_stage2_data_n[0:NUM_GAUSSIANS-1]; 
    // assign debug_stage2_data = conv_matrix;
    reg [4*BIT_WIDTH-1:0] debug_stage3_data[0:NUM_GAUSSIANS-1];
    reg [4*BIT_WIDTH-1:0] debug_stage3_data_n[0:NUM_GAUSSIANS-1];
    // assign debug_stage3_data = campos;
    reg [6*BIT_WIDTH-1:0] debug_bbox_data[0:NUM_GAUSSIANS-1];
    reg [6*BIT_WIDTH-1:0] debug_bbox_data_n[0:NUM_GAUSSIANS-1];
    // assign debug_bbox_data = bbox_info;
    reg [6*BIT_WIDTH-1:0] debug_stage4_data[0:NUM_GAUSSIANS-1];
    reg [6*BIT_WIDTH-1:0] debug_stage4_data_n[0:NUM_GAUSSIANS-1];
    // assign debug_stage4_data = jacobian;
    reg [6*BIT_WIDTH-1:0] debug_stage5_data[0:NUM_GAUSSIANS-1];
    reg [6*BIT_WIDTH-1:0] debug_stage5_data_n[0:NUM_GAUSSIANS-1];
    // assign debug_stage5_data = conv2d;
    reg [3*BIT_WIDTH-1:0] debug_stage6_data[0:NUM_GAUSSIANS-1];
    reg [3*BIT_WIDTH-1:0] debug_stage6_data_n[0:NUM_GAUSSIANS-1];
    // assign debug_stage6_data = conic_coeff;
    reg [4*BIT_WIDTH-1:0] debug_stage7_data[0:NUM_GAUSSIANS-1];
    reg [4*BIT_WIDTH-1:0] debug_stage7_data_n [0:NUM_GAUSSIANS-1]; 
    // assign debug_stage7_data = sort_list;

    // sequential debug signal (check correctness)
    always@(posedge clk)begin
        if(~srst_n)begin
            for(i=0;i<NUM_GAUSSIANS;i=i+1)begin
                debug_stage1_data[i] <= 0;
                debug_stage2_data[i] <= 0;
                debug_stage3_data[i] <= 0;
                debug_bbox_data[i] <= 0;
                debug_stage4_data[i] <= 0;
                debug_stage5_data[i] <= 0;
                debug_stage6_data[i] <= 0;
                debug_stage7_data[i] <= 0;
            end 
        end else begin 
            for(i=0;i<NUM_GAUSSIANS;i=i+1)begin
                debug_stage1_data[i] <= debug_stage1_data_n[i];
                debug_stage2_data[i] <= debug_stage2_data_n[i];
                debug_stage3_data[i] <= debug_stage3_data_n[i];
                debug_bbox_data[i] <= debug_bbox_data_n[i];
                debug_stage4_data[i] <= debug_stage4_data_n[i];
                debug_stage5_data[i] <= debug_stage5_data_n[i];
                debug_stage6_data[i] <= debug_stage6_data_n[i];
                debug_stage7_data[i] <= debug_stage7_data_n[i];
            end 
        end
    end
    // combinational debug signal (check correctness)
    always@(*)begin
        // assign default value
        for(i=0;i<NUM_GAUSSIANS;i=i+1)begin
            debug_stage1_data_n[i] = debug_stage1_data[i];
            debug_stage2_data_n[i] = debug_stage2_data[i];
            debug_stage3_data_n[i] = debug_stage3_data[i];
            debug_bbox_data_n[i] = debug_bbox_data[i];
            debug_stage4_data_n[i] = debug_stage4_data[i];
            debug_stage5_data_n[i] = debug_stage5_data[i];
            debug_stage6_data_n[i] = debug_stage6_data[i];
            debug_stage7_data_n[i] = debug_stage7_data[i];
        end
        // assign debug signal for this round
        debug_stage1_data_n[gaussian_cnt] = rotation_matrix; 
        debug_stage2_data_n[gaussian_cnt] = conv_matrix;
        debug_stage3_data_n[gaussian_cnt] = campos;
        debug_bbox_data_n[gaussian_cnt] = bbox_info;
        debug_stage4_data_n[gaussian_cnt] = jacobian;
        debug_stage5_data_n[gaussian_cnt] = conv2d;
        debug_stage6_data_n[gaussian_cnt] = conic_coeff;
        debug_stage7_data_n[gaussian_cnt] = sort_list;
    end

    */
    //========================
    // SEQUENTIAL LOGIC
    //========================
    always@(posedge clk)begin 
        if(~srst_n)begin 
            enable_reg <= 0;
            sram_rdata_input_reg <= 0;
            sram_rdata_view_reg <= 0;
            sram_rdata_img_reg <= 0;
            sram_rdata_sort_reg <= 0;
            sram_rdata_raster_reg <= 0;
            // sram_rdata_temp_reg <= 0;
            valid <= 0;
            sram_wdata_input <= 0;
            sram_raddr_input <= 0;
            sram_waddr_input <= 0;
            sram_wen_input <= 1;
            sram_wdata_view <= 0;
            sram_raddr_view <= 0;
            sram_waddr_view <= 0;
            sram_wen_view <= 1;
            sram_wdata_img <= 0;
            sram_raddr_img <= 0;
            sram_waddr_img <= 0;
            sram_wen_img <= 1;
            sram_wdata_sort <= 0;
            sram_raddr_sort <= 0;
            sram_waddr_sort <= 0;
            sram_wen_sort <= 1;
            sram_wdata_raster <= 0;
            sram_raddr_raster <= 0;
            sram_waddr_raster <= 0;
            sram_wen_raster <= 1;
            sram_wordmask_raster <= 0;
            // sram_wdata_temp <= 0;
            // sram_raddr_temp <= 0;
            // sram_waddr_temp <= 0;
            // sram_wen_temp <= 1;
        end else begin 
            // input ff 
            enable_reg <= enable;
            sram_rdata_input_reg <= sram_rdata_input;
            sram_rdata_view_reg <= sram_rdata_view;
            sram_rdata_img_reg <= sram_rdata_img;
            sram_rdata_sort_reg <= sram_rdata_sort;
            sram_rdata_raster_reg <= sram_rdata_raster;
            // sram_rdata_temp_reg <= sram_rdata_temp;
            // output ff
            valid <= valid_n;
            sram_wdata_input <= sram_wdata_input_n;
            sram_raddr_input <= sram_raddr_input_n;
            sram_waddr_input <= sram_waddr_input_n;
            sram_wen_input <= sram_wen_input_n;
            sram_wdata_view <= sram_wdata_view_n;
            sram_raddr_view <= sram_raddr_view_n;
            sram_waddr_view <= sram_waddr_view_n;
            sram_wen_view <= sram_wen_view_n;
            sram_wdata_img <= sram_wdata_img_n; //以後會用到
            sram_raddr_img <= sram_raddr_img_n;
            sram_waddr_img <= sram_waddr_img_n;
            sram_wen_img <= sram_wen_img_n;
            sram_wdata_sort <= sram_wdata_sort_n;
            sram_raddr_sort <= sram_raddr_sort_n;
            sram_waddr_sort <= sram_waddr_sort_n;
            sram_wen_sort <= sram_wen_sort_n;
            sram_wdata_raster <= sram_wdata_raster_n;
            sram_raddr_raster <= sram_raddr_raster_n;
            sram_waddr_raster <= sram_waddr_raster_n;
            sram_wen_raster <= sram_wen_raster_n;
            sram_wordmask_raster <= sram_wordmask_raster_n;
            // sram_wdata_temp <= sram_wdata_temp_n;
            // sram_raddr_temp <= sram_raddr_temp_n;
            // sram_waddr_temp <= sram_waddr_temp_n;
            // sram_wen_temp <= sram_wen_temp_n;
        end
    end


    // ========================
    // FSM
    // =========================

    localparam  IDLE = 4'b0000, 
                ROT  = 4'b0001, 
                COV3D = 4'b0010, 
                CAMPOS = 4'b0011, 
                JACOBIAN = 4'b0100, 
                COV2D = 4'b0101, 
                CONIC = 4'b0110, 
                SORT = 4'b0111, 
                RENDER = 4'b1000, 
                DONE = 4'b1001;

    fsm U_FSM(
        .clk(clk),
        .srst_n(srst_n),
        .enable(enable_reg),
        .rot_valid(rot_valid),
        .conv_valid(conv_valid),
        .campos_valid(campos_valid),
        .jacobian_valid(jacobian_valid),
        .conv2d_valid(conv2d_valid),
        .sort_valid(sort_valid),
        .render_valid(render_valid),
        .gaussian_cnt(gaussian_cnt),
        .gaussian_num(gaussian_num),
        .state(state),
        .rot_enable(rot_enable),
        .conv_enable(conv_enable),
        .campos_enable(campos_enable),
        .jacobian_enable(jacobian_enable),
        .conv2d_enable(conv2d_enable),
        .conic_valid(conic_valid),
        .conic_enable(conic_enable),
        .sort_enable(sort_enable), 
        .render_enable(render_enable)
    );

    //========================
    // OUTPUT SELECTION
    //======================== 
    // 選擇sram的訊號
    always@(*)begin 
        sram_wen_input_n = 1;
        sram_wen_view_n = 1;
        sram_wen_img_n = 1;
        sram_wdata_input_n = 0;
        sram_raddr_input_n = 0;
        sram_waddr_input_n = 0;
        sram_wdata_view_n = 0;
        sram_raddr_view_n = 0;
        sram_waddr_view_n = 0;
        sram_wdata_img_n = 0;
        sram_raddr_img_n = 0;
        sram_waddr_img_n = 0;
        sram_wdata_sort_n = 0;
        sram_raddr_sort_n = 0;
        sram_waddr_sort_n = 0;
        sram_wen_sort_n = 1;
        sram_raddr_raster_n = 0;
        sram_waddr_raster_n = 0;
        sram_wdata_raster_n = 0;
        sram_wordmask_raster_n = 16'b0;
        sram_wen_raster_n = 1;
        case(state)
            ROT :begin 
                sram_wdata_input_n = rot_sram_wdata_input;
                sram_raddr_input_n = rot_sram_raddr_input;
                sram_waddr_input_n = rot_sram_waddr_input;
            end
            COV3D :begin 
                sram_wdata_input_n = conv_sram_wdata_input;
                sram_raddr_input_n = conv_sram_raddr_input;
                sram_waddr_input_n = conv_sram_waddr_input;
            end
            CAMPOS : begin 
                sram_wdata_input_n = campos_sram_wdata_input;
                sram_raddr_input_n = campos_sram_raddr_input;
                sram_waddr_input_n = campos_sram_waddr_input;
                sram_wdata_view_n = campos_sram_wdata_view;
                sram_raddr_view_n = campos_sram_raddr_view;
                sram_waddr_view_n = campos_sram_waddr_view;
                sram_wen_sort_n = campos_sram_wen_sort;
                sram_waddr_sort_n = campos_sram_waddr_sort;
                sram_wdata_sort_n = campos_sram_wdata_sort;
                //sram_raddr_sort_n = campos_sram_raddr_sort;
            end
            // Jacobian stage don't need any memory access
            COV2D : begin 
                //sram_wdata_view_n = conv2d_sram_wdata_view;
                sram_raddr_view_n = conv2d_sram_raddr_view;
                //sram_waddr_view_n = conv2d_sram_waddr_view;
                sram_waddr_raster_n = conv2d_sram_waddr_raster;
                sram_wdata_raster_n = conv2d_sram_wdata_raster;
                sram_wordmask_raster_n = conv2d_sram_wordmask_raster;
                sram_wen_raster_n = conv2d_sram_wen_raster;
            end
            CONIC: begin
                sram_waddr_raster_n = conic_sram_waddr_raster;
                sram_wdata_raster_n = conic_sram_wdata_raster;
                sram_wordmask_raster_n = conic_sram_wordmask_raster;
                sram_wen_raster_n = conic_sram_wen_raster; 
            end
            SORT: begin 
                // TODO : add sort sram access
                sram_wdata_sort_n = sort_sram_wdata_sort;
                sram_raddr_sort_n = sort_sram_raddr_sort;
                sram_waddr_sort_n = sort_sram_waddr_sort;
                sram_wen_sort_n = sort_sram_wen_sort;
            end
            RENDER : begin
                //sram_wdata_input_n = render_sram_wdata_input;
                sram_raddr_input_n = render_sram_raddr_input;
                //sram_waddr_input_n = render_sram_waddr_input;
                sram_wdata_img_n = render_sram_wdata_img;
                sram_raddr_img_n = render_sram_raddr_img;
                sram_waddr_img_n = render_sram_waddr_img;
                sram_wen_img_n = render_sram_wen_img; 
                sram_raddr_raster_n = render_sram_raddr_raster;
                // sram_wdata_temp_n = render_sram_wdata_temp;
                // sram_raddr_temp_n = render_sram_raddr_temp;
                // sram_waddr_temp_n = render_sram_waddr_temp;
                // sram_wen_temp_n = render_sram_wen_temp;
                sram_raddr_sort_n = render_sram_raddr_sort;
            end
            default:;
        endcase
    end
    // output 
    assign valid_n = (state==DONE);

    //========================
    // MULTIPLY SELECTION
    //======================== 
    always @* begin
        multiplicand_0 = 0;
        multiplicand_long_0 = 0;
        multiplier_0 = 0;
        multiplier_long_0 = 0;
        case (state)
            ROT : begin
                multiplicand_0 = rot_multiplicand;
                multiplier_0 = rot_multiplier;
            end

            COV3D: begin
                multiplicand_0 = conv_multiplicand;
                multiplier_0 = conv_multiplier;
            end

            CAMPOS: begin
                multiplicand_0 = campos_multiplicand;
                multiplier_0 = campos_multiplier;
            end

            JACOBIAN: begin
                multiplicand_0 = jacobian_multiplicand;
                multiplier_0 = jacobian_multiplier;
            end

            COV2D: begin
                multiplicand_0 = conv2d_multiplicand;
                multiplicand_long_0 = conv2d_multiplicand_long;
                multiplier_0 = conv2d_multiplier;
                multiplier_long_0 = conv2d_multiplier_long;
            end 

            CONIC: begin
                multiplicand_0 = conic_multiplicand;
                multiplier_0 = conic_multiplier;
                multiplicand_long_0 = conic_multiplicand_long;
                multiplier_long_0 = conic_multiplier_long;
            end

            RENDER: begin
                multiplicand_0 = render_multiplicand;
                multiplier_0 = render_multiplier;
                multiplicand_long_0 = render_multiplicand_long;
                multiplier_long_0 = render_multiplier_long;
            end

            default:;
        endcase
    end

    //========================
    // DIVISION SELECTION
    //======================== 
    always @* begin
        dividend = 0;
        divisor = 0;
        case (state)
            JACOBIAN: begin
                dividend = jacobian_dividend;
                divisor = jacobian_divisor;
            end
            CONIC: begin
                dividend = conic_dividend;
                divisor = conic_divisor;
            end
            default:;
        endcase
    end

    //========================
    // SUBMODULEs
    //======================== 


    Stage1_Rot #(
        .BIT_WIDTH(BIT_WIDTH), 
        .FLOAT_WIDTH(6),
        .PRODUCT_WIDTH(PRODUCT_WIDTH)
    ) stage1_rot (
        .clk(clk),
        .srst_n(srst_n),
        .enable(rot_enable),
        .gaussian_cnt(gaussian_cnt),
        .rot_product(product_0),
        .valid(rot_valid),
        .sram_rdata_input(sram_rdata_input_reg),
        .sram_wdata_input(rot_sram_wdata_input),
        .sram_raddr_input(rot_sram_raddr_input),
        .sram_waddr_input(rot_sram_waddr_input),
        .rotation_matrix(rotation_matrix),
        .rot_multiplicand(rot_multiplicand),
        .rot_multiplier(rot_multiplier)
    );

    Stage2_Conv #(
        .BIT_WIDTH(BIT_WIDTH),
        .FLOAT_WIDTH(6),
        .PRODUCT_WIDTH(PRODUCT_WIDTH)
    ) stage2_conv (
        .clk(clk),
        .srst_n(srst_n),
        .enable(conv_enable),
        .gaussian_cnt(gaussian_cnt),
        .conv_product(product_0),
        .valid(conv_valid),
        .sram_rdata_input(sram_rdata_input_reg),
        .sram_wdata_input(conv_sram_wdata_input),
        .sram_raddr_input(conv_sram_raddr_input),
        .sram_waddr_input(conv_sram_waddr_input),
        .rotation_matrix(rotation_matrix),
        .conv_matrix(conv_matrix),
        .conv_multiplicand(conv_multiplicand),
        .conv_multiplier(conv_multiplier)
    );  

    Stage3_Campos #(
        .BIT_WIDTH(BIT_WIDTH),
        .FLOAT_WIDTH(6),
        .PRODUCT_WIDTH(PRODUCT_WIDTH)
    ) stage3_campos (
        .clk(clk),
        .srst_n(srst_n),
        .enable(campos_enable),
        .valid(campos_valid),
        .gaussian_cnt(gaussian_cnt),
        .gaussian_num(gaussian_num), 
        .campos_product(product_0),
        .valid_gaussian_num(valid_gaussian_num),
        // sram interface (gaussian splatting)
        .sram_rdata_input(sram_rdata_input_reg),
        .sram_wdata_input(campos_sram_wdata_input),
        .sram_raddr_input(campos_sram_raddr_input),
        .sram_waddr_input(campos_sram_waddr_input),
        // sram interface (sort)
        //.sram_rdata_sort(sram_rdata_sort_reg),
        .sram_wdata_sort(campos_sram_wdata_sort),
        //.sram_raddr_sort(campos_sram_raddr_sort),
        .sram_waddr_sort(campos_sram_waddr_sort),
        .sram_wen_sort(campos_sram_wen_sort),
        // sram interface (view)
        .sram_rdata_view(sram_rdata_view_reg),
        .sram_wdata_view(campos_sram_wdata_view),
        .sram_raddr_view(campos_sram_raddr_view),
        .sram_waddr_view(campos_sram_waddr_view),
        // camera position 
        .campos(campos),
        .campos_multiplicand(campos_multiplicand),
        .campos_multiplier(campos_multiplier)
    );

    Stage4_Jacobian #(
        .BIT_WIDTH(BIT_WIDTH), 
        .FLOAT_WIDTH(6),
        .PRODUCT_WIDTH(PRODUCT_WIDTH),
        .RENDER_WIDTH(RENDER_WIDTH)
    ) stage4_jacobian (
        .clk(clk),
        .srst_n(srst_n),
        .enable(jacobian_enable),
        .jacobian_product(product_0),
        .jacobian_quotient(quotient),
        .valid(jacobian_valid),
        // camera position 
        .campos(campos),
        // jacobian 
        .jacobian(jacobian),
        .jacobian_multiplicand(jacobian_multiplicand),
        .jacobian_multiplier(jacobian_multiplier),
        .jacobian_dividend(jacobian_dividend),
        .jacobian_divisor(jacobian_divisor)
    );

    Stage5_Conv2D #(
        .BIT_WIDTH(BIT_WIDTH), 
        .LONG_BIT_WIDTH(LONG_BIT_WIDTH),
        .FLOAT_WIDTH(6),
        .PRODUCT_WIDTH(PRODUCT_WIDTH),
        .PRODUCT_WIDTH_LONG(PRODUCT_WIDTH_LONG),
        .RENDER_WIDTH(RENDER_WIDTH),
        .RENDER_HEIGHT(RENDER_HEIGHT)
    ) stage5_conv2d (
        .clk(clk),
        .srst_n(srst_n),
        .enable(conv2d_enable),
        .gaussian_cnt(gaussian_cnt),
        .conv2d_product(product_0),
        .conv2d_product_long(product_long_0),
        .valid(conv2d_valid),
        // sram interface 
        .sram_rdata_view(sram_rdata_view_reg),
        //.sram_wdata_view(conv2d_sram_wdata_view),
        .sram_raddr_view(conv2d_sram_raddr_view),
        //.sram_waddr_view(conv2d_sram_waddr_view),
        // sram interface (raster)
        .sram_wdata_raster(conv2d_sram_wdata_raster),
        .sram_waddr_raster(conv2d_sram_waddr_raster),
        .sram_wordmask_raster(conv2d_sram_wordmask_raster),
        .sram_wen_raster(conv2d_sram_wen_raster),
        // jacobian 
        .jacobian(jacobian),
        // conv matrix 
        .conv_matrix(conv_matrix),
        // campos 
        .campos(campos),
        // conv2d 
        .conv2d(conv2d),
        .conv2d_multiplicand(conv2d_multiplicand),
        .conv2d_multiplicand_long(conv2d_multiplicand_long),
        .conv2d_multiplier(conv2d_multiplier),
        .conv2d_multiplier_long(conv2d_multiplier_long),

        // bbox info 
        .bbox_info(bbox_info)
    );

    Stage6_Conic #(
        .BIT_WIDTH(BIT_WIDTH), 
        .LONG_BIT_WIDTH(LONG_BIT_WIDTH),
        .FLOAT_WIDTH(6),
        .PRODUCT_WIDTH(PRODUCT_WIDTH),
        .PRODUCT_WIDTH_LONG(PRODUCT_WIDTH_LONG)
    ) stage6_conic (
        .clk(clk),
        .srst_n(srst_n),
        .enable(conic_enable),
        .gaussian_cnt(gaussian_cnt),
        .conic_product(product_0),
        .conic_product_long(product_long_0),
        .conic_quotient(quotient),
        .valid(conic_valid),
        .conv2d(conv2d),
        .conic_coeff(conic_coeff),
        .conic_multiplicand(conic_multiplicand),
        .conic_multiplier(conic_multiplier),
        .conic_multiplicand_long(conic_multiplicand_long),
        .conic_multiplier_long(conic_multiplier_long),
        .conic_dividend(conic_dividend),
        .conic_divisor(conic_divisor), 
        // sram interface 
        .sram_waddr_raster(conic_sram_waddr_raster),
        .sram_wdata_raster(conic_sram_wdata_raster),
        .sram_wordmask_raster(conic_sram_wordmask_raster),
        .sram_wen_raster(conic_sram_wen_raster)
    );

    Stage7_Sort #(
        .BIT_WIDTH(BIT_WIDTH)
    ) stage7_sort (
        .clk(clk),
        .srst_n(srst_n),
        .enable(sort_enable),
        .valid(sort_valid),
        .gaussian_num(gaussian_num),
        // .depth(campos[BIT_WIDTH-1:0]), // campos depth
        .sram_rdata_sort(sram_rdata_sort_reg),
        .sort_list(sort_list), 
        .sram_wdata_sort(sort_sram_wdata_sort),
        .sram_raddr_sort(sort_sram_raddr_sort),
        .sram_waddr_sort(sort_sram_waddr_sort),
        .sram_wen_sort(sort_sram_wen_sort)
    );

    Stage8_Render #(
        .BIT_WIDTH(BIT_WIDTH), 
        .LONG_BIT_WIDTH(LONG_BIT_WIDTH),
        .PRODUCT_WIDTH(PRODUCT_WIDTH),
        .PRODUCT_WIDTH_LONG(PRODUCT_WIDTH_LONG),
        .RENDER_WIDTH(RENDER_WIDTH), 
        .RENDER_HEIGHT(RENDER_HEIGHT),
        .RENDER_WIDTH_LOG(RENDER_WIDTH_LOG)
    ) stage8_render (
        .clk(clk),
        .srst_n(srst_n),
        .enable(render_enable),
        .render_product(product_0),
        .render_product_long(product_long_0),
        .gaussian_num(valid_gaussian_num),

        .valid(render_valid),

        // SRAM input interface 
        .sram_rdata_input(sram_rdata_input_reg),
        .sram_raddr_input(render_sram_raddr_input),
        
        // Output Image
        .sram_rdata_img(sram_rdata_img_reg),
        .sram_wdata_img(render_sram_wdata_img),
        .sram_raddr_img(render_sram_raddr_img),
        .sram_waddr_img(render_sram_waddr_img),
        .sram_wen_img(render_sram_wen_img),

        // Temp Image
        // .sram_rdata_temp(sram_rdata_temp_reg),
        // .sram_wdata_temp(render_sram_wdata_temp),
        // .sram_raddr_temp(render_sram_raddr_temp),
        // .sram_waddr_temp(render_sram_waddr_temp),
        // .sram_wen_temp(render_sram_wen_temp),

        // Raster
        .sram_rdata_raster(sram_rdata_raster_reg),
        .sram_raddr_raster(render_sram_raddr_raster),

        // Sort 
        .sram_rdata_sort(sram_rdata_sort_reg),
        .sram_raddr_sort(render_sram_raddr_sort),

        .render_multiplicand(render_multiplicand),
        .render_multiplier(render_multiplier),
        .render_multiplicand_long(render_multiplicand_long),
        .render_multiplier_long(render_multiplier_long)
    );

multiply #(
    .BW_PER_MCD (BIT_WIDTH),
    .BW_PER_MER (BIT_WIDTH),
    .N_MULT (N_MULT)
) U_MULTIPLY_0 (
    .clk(clk),
    .in_a(multiplicand_0),
    .in_b(multiplier_0),
    .out(product_0)
);

multiply_long #(
    .BW_PER_MCD (LONG_BIT_WIDTH),
    .BW_PER_MER (BIT_WIDTH),
    .N_MULT (N_MULT)
) U_MULTIPLY_LONG_0 (
    .clk(clk),
    .in_a(multiplicand_long_0),
    .in_b(multiplier_long_0),
    .out(product_long_0)
);


division #(
    .A_WIDTH(36),  
    .B_WIDTH(36)
) U_DIVISION (
    .clk(clk),
    .srst_n(srst_n),
    .a(dividend),
    .b(divisor),
    .quotient(quotient), 
    .divide_by_0(divide_by_0)
);

endmodule
