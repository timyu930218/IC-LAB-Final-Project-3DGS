//======================================================================================================
//  Note:          Use only for teaching materials of IC Design Lab, NTHU.
//  Copyright: (c) 2025 Vision Circuits and Systems Lab, NTHU, Taiwan. ALL Rights Reserved.
//======================================================================================================

`timescale 1ns/100ps

`define PAT_L 0
`define PAT_U 0
`define NUM_PAT (`PAT_U-`PAT_L+1)

`define PAT_NAME_LENGTH 2
`define CYCLE 10
`define END_CYCLES 40000000
`define FLAG_VERBOSE 1  
`define FLAG_DUMPWV 1

module test_top_128;

// Parameters
localparam BW_PER_PARAM = 32;
localparam BW_PER_ACT = 32;

// Stage Definitions
localparam STAGE1_ROT = 0;
localparam STAGE2_SCALE = 1;
localparam STAGE3_CAM_POS = 2;
localparam STAGE4_JACOBIAN = 3;
localparam STAGE5_COV2D = 4;
localparam STAGE6_CONIC = 5;
localparam STAGE7_SORT = 6;
localparam STAGE8_RENDER = 7;
localparam STAGE_BBOX = 8;

localparam STAGE_RASTER_PARAM = 9;

integer test_stage;
reg [8*20-1:0] stage_str;

initial begin
    stage_str = 0;
    `ifdef STAGE1_ROT
        test_stage = STAGE1_ROT;
        stage_str = "Stage 1: Rotation";
    `elsif STAGE2_SCALE
        test_stage = STAGE2_SCALE;
        stage_str = "Stage 2: Scale";
    `elsif STAGE3_CAM_POS
        test_stage = STAGE3_CAM_POS;
        stage_str = "Stage 3: Cam Pos";
    `elsif STAGE4_JACOBIAN
        test_stage = STAGE4_JACOBIAN;
        stage_str = "Stage 4: Jacobian";
    `elsif STAGE5_COV2D
        test_stage = STAGE5_COV2D;
        stage_str = "Stage 5: Cov2D";
    `elsif STAGE6_CONIC
        test_stage = STAGE6_CONIC;
        stage_str = "Stage 6: Conic";
    `elsif STAGE7_SORT
        test_stage = STAGE7_SORT;
        stage_str = "Stage 7: Sort";
    `elsif STAGE8_RENDER
        test_stage = STAGE8_RENDER;
        stage_str = "Stage 8: Render";
    `elsif STAGE_BBOX
        test_stage = STAGE_BBOX;
        stage_str = "Stage: Bounding Box";
    `elsif STAGE_RASTER_PARAM
        test_stage = STAGE_RASTER_PARAM;
        stage_str = "Stage: Raster Param";
    `else
        test_stage = STAGE8_RENDER; // Default
        stage_str = "Stage 8: Render";
    `endif
end

integer i;
// ===== pattern files ===== // 
reg [60*8-1:0] input_gauss_file;
reg [60*8-1:0] input_view_file;

reg [60*8-1:0] golden_rot_file;
reg [60*8-1:0] golden_scale_file;
reg [60*8-1:0] golden_cam_pos_file;
reg [60*8-1:0] golden_jacobian_file;
reg [60*8-1:0] golden_cov2d_file;
reg [60*8-1:0] golden_conic_file;
reg [60*8-1:0] golden_sort_file;
reg [60*8-1:0] golden_image_file;
reg [60*8-1:0] golden_bbox_file;
reg [60*8-1:0] golden_raster_param_file;

// ===== module I/O ===== //
reg clk;
reg srst_n;
reg enable;
wire valid;

// SRAM Interface (Input)
wire [63:0] sram_rdata_input;
wire [15:0] sram_raddr_input;
wire sram_wen_input;
wire [63:0] sram_wdata_input;
wire [15:0] sram_waddr_input;

// SRAM Interface (View)
wire [63:0] sram_rdata_view;
wire [15:0] sram_raddr_view;
wire sram_wen_view;
wire [63:0] sram_wdata_view;
wire [15:0] sram_waddr_view;

// SRAM Interface (Output Image)
wire [63:0] sram_rdata_img;
wire [15:0] sram_raddr_img;
wire sram_wen_img;
wire [63:0] sram_wdata_img;
wire [15:0] sram_waddr_img;

// SRAM Interface (Output Temp)
// wire [63:0] sram_rdata_temp;
// wire [15:0] sram_raddr_temp;
// wire sram_wen_temp;
// wire [63:0] sram_wdata_temp;
// wire [15:0] sram_waddr_temp;

// SRAM Interface (Sort List)
wire [31:0] sram_rdata_sort;
wire [5:0] sram_raddr_sort;
wire sram_wen_sort;
wire [31:0] sram_wdata_sort;
wire [5:0] sram_waddr_sort;
reg [6:0] gaussian_num; 

// SRAM Interface (Raster Param) - 128 bit
wire [127:0] sram_rdata_raster;
wire [5:0] sram_raddr_raster;
wire sram_wen_raster;
wire [127:0] sram_wdata_raster;
wire [5:0] sram_waddr_raster;
wire [15:0] sram_wordmask_raster;

// Instantiate DUT
Gaussian_Splatting_Top #(
    .RENDER_WIDTH(128),
    .RENDER_HEIGHT(128),
    .RENDER_WIDTH_LOG(7)
) uut (
    .clk(clk),
    .srst_n(srst_n),
    .enable(enable),
    .valid(valid),
    .gaussian_num(gaussian_num),
    
    // Input SRAM
    .sram_rdata_input(sram_rdata_input),
    .sram_raddr_input(sram_raddr_input),
    .sram_wen_input(sram_wen_input),
    .sram_wdata_input(sram_wdata_input),
    .sram_waddr_input(sram_waddr_input),
    
    // View SRAM
    .sram_rdata_view(sram_rdata_view),
    .sram_raddr_view(sram_raddr_view),
    .sram_wen_view(sram_wen_view),
    .sram_wdata_view(sram_wdata_view),
    .sram_waddr_view(sram_waddr_view),
    
    // Sort List SRAM
    .sram_rdata_sort(sram_rdata_sort),
    .sram_raddr_sort(sram_raddr_sort),
    .sram_wen_sort(sram_wen_sort),
    .sram_wdata_sort(sram_wdata_sort),
    .sram_waddr_sort(sram_waddr_sort),

    // Output SRAM
    .sram_rdata_img(sram_rdata_img),
    .sram_raddr_img(sram_raddr_img),
    .sram_wen_img(sram_wen_img),
    .sram_wdata_img(sram_wdata_img),
    .sram_waddr_img(sram_waddr_img),

    // Temp SRAM
    // .sram_rdata_temp(sram_rdata_temp),
    // .sram_raddr_temp(sram_raddr_temp),
    // .sram_wen_temp(sram_wen_temp),
    // .sram_wdata_temp(sram_wdata_temp),
    // .sram_waddr_temp(sram_waddr_temp),

    // Raster Param SRAM
    .sram_rdata_raster(sram_rdata_raster),
    .sram_raddr_raster(sram_raddr_raster),
    .sram_wen_raster(sram_wen_raster),
    .sram_wdata_raster(sram_wdata_raster),
    .sram_waddr_raster(sram_waddr_raster),
    .sram_wordmask_raster(sram_wordmask_raster)
);

// SRAM Models
// Input Gaussian Data (1024 x 64)
sram_generic #(
    .DATA_WIDTH(64),
    .ADDR_WIDTH(10),
    .DEPTH(1024)
) sram_input_u (
    .clk(clk), 
    .wordmask(8'd0), 
    .csb(1'b0), 
    .wsb(sram_wen_input), 
    .wdata(sram_wdata_input),  
    .waddr(sram_waddr_input[9:0]),  
    .raddr(sram_raddr_input[9:0]),  
    .rdata(sram_rdata_input)
);

// View Matrix (16 x 32)
sram_generic #(
    .DATA_WIDTH(64),
    .ADDR_WIDTH(4),
    .DEPTH(4)
) sram_view_u (
    .clk(clk),
    .wordmask(8'd0),
    .csb(1'b0), 
    .wsb(sram_wen_view), 
    .wdata(sram_wdata_view), 
    .waddr(sram_waddr_view[3:0]), 
    .raddr(sram_raddr_view[3:0]), 
    .rdata(sram_rdata_view)
);

// Output Image (16384 x 32) - Updated for 128x128
sram_generic #(
    .DATA_WIDTH(64),
    .ADDR_WIDTH(14),
    .DEPTH(16384)
) sram_img_u (
    .clk(clk), 
    .wordmask(8'd0),
    .csb(1'b0), 
    .wsb(sram_wen_img), 
    .wdata(sram_wdata_img), 
    .waddr(sram_waddr_img[13:0]),  
    .raddr(sram_raddr_img[13:0]),  
    .rdata(sram_rdata_img) 
);

// Output TEMP (16384 x 32) - Updated for 128x128
// sram_generic #(
//     .DATA_WIDTH(64),
//     .ADDR_WIDTH(14),
//     .DEPTH(16384)
// ) sram_temp_u (
//     .clk(clk), 
//     .wordmask(8'd0),
//     .csb(1'b0), 
//     .wsb(sram_wen_temp), 
//     .wdata(sram_wdata_temp), 
//     .waddr(sram_waddr_temp[13:0]),  
//     .raddr(sram_raddr_temp[13:0]),  
//     .rdata(sram_rdata_temp) 
// );

// Sort List SRAM (64)
sram_generic #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(6),
    .DEPTH(64)
) sram_sort_u (
    .clk(clk), 
    .wordmask(4'd0),
    .csb(1'b0), 
    .wsb(sram_wen_sort), 
    .wdata(sram_wdata_sort), 
    .waddr(sram_waddr_sort),  
    .raddr(sram_raddr_sort),  
    .rdata(sram_rdata_sort) 
);



// Raster Param SRAM (64 x 128)
sram_generic #(
    .DATA_WIDTH(128),
    .ADDR_WIDTH(6),
    .DEPTH(64)
) sram_raster_u (
    .clk(clk), 
    .wordmask(sram_wordmask_raster),
    .csb(1'b0), 
    .wsb(sram_wen_raster), 
    .wdata(sram_wdata_raster), 
    .waddr(sram_waddr_raster),  
    .raddr(sram_raddr_raster),  
    .rdata(sram_rdata_raster) 
);  


initial begin
`ifdef GATESIM
    //$fsdbDumpfile("gatesim_gaussian_splatting_128.fsdb");
    //$fsdbDumpvars("+mda");
	$sdf_annotate("../../../SYN/Gaussian_Splatting_Top_syn.sdf",uut);
`else
    $fsdbDumpfile("presim_gaussian_splatting_128.fsdb");
    $fsdbDumpvars("+mda");
`endif
end

// ===== parameter & golden answers ===== //
// ===== parameter & golden answers ===== //
reg [223:0] input_gauss_mem [0:1024-1]; // 14 words * 16 bits = 224 bits
reg [255:0] input_view_mem [0:0];
reg [63:0] golden_img_mem [0:16384-1];

// Intermediate Golden Data

// Intermediate Golden Data


// ===== system reset ===== //
initial begin
    clk = 0;
    while(1) #(`CYCLE/2) clk = ~clk;
end

initial begin
  #(`CYCLE * `END_CYCLES);
    $display("\n========================================================");
    $display("   Error!!! Simulation time is too long...            ");
    $display("========================================================");
    $finish;
end

// ===== cycle counter ===== //
integer cycle_cnt;
initial begin
    cycle_cnt = 0;
    while(1) begin 
        cycle_cnt = cycle_cnt + 1;
        @(negedge clk);
    end
end

// ===== output comparision ===== //
integer pat_idx;
integer total_err_pat;
integer err_cnt;
integer num_gaussians;

initial begin
    // Check PAT_L and PAT_U
    if((`PAT_L < 0) || (`PAT_L > 149) || (`PAT_U < 0) || (`PAT_U > 149)) begin
        $display("Error: PAT_L/PAT_U out of range");
        $finish;
    end

    $display("\nStart checking %s ...\n", stage_str);

    total_err_pat = 0;
    for(pat_idx=`PAT_L; pat_idx<=`PAT_U; pat_idx=pat_idx+1) begin
        
        // Determine number of Gaussians based on pattern index
        // Patterns 0-49 -> 1 Gaussian, 50-99 -> 4 Gaussians
        if (pat_idx < 50) num_gaussians = 1;
        else if (pat_idx < 100) num_gaussians = 4;
        else num_gaussians = 64;

        if (pat_idx < 50) gaussian_num = 1;
        else if (pat_idx < 100) gaussian_num = 4;
        else gaussian_num = 64; 

        // Load Input Data
        load_input(pat_idx, num_gaussians);
        
        // Load Golden Data based on stage
        load_golden(pat_idx, test_stage);

        $display("\nPattern %02d", pat_idx);

        srst_n = 1;
        enable = 0;
        @(negedge clk); srst_n = 1'b0;
        @(negedge clk); srst_n = 1'b1; enable = 1'b1;
        @(negedge clk); enable = 1'b0;
        
        // Wait for completion
        wait(valid);
        @(negedge clk);
        
        // Verify
        err_cnt = 0;
        case (test_stage)
            STAGE1_ROT: verify_stage1();
            STAGE2_SCALE: verify_stage2();
            STAGE3_CAM_POS: verify_stage3();
            STAGE4_JACOBIAN: verify_stage4();
            STAGE5_COV2D: verify_stage5();
            STAGE6_CONIC: verify_stage6();
            STAGE7_SORT: verify_stage7();
            STAGE8_RENDER: verify_render();
            STAGE_BBOX: verify_bbox();
            STAGE_RASTER_PARAM: verify_raster_param();
        endcase
        
        if(err_cnt > 0) begin
            $display("Pattern %02d FAILED with %d errors", pat_idx, err_cnt);
            total_err_pat = total_err_pat + 1;
        end else begin
            $display("Pattern %02d PASSED", pat_idx);
        end
        
    end // for pat_idx

    if(total_err_pat == 0) begin 
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠟⢛⣛⠛⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⡿⣿⣿⢏⢸⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠋⣡⣴⣾⣿⣿⣿⡄⠘⣿⣿⣿⡿⠟⠛⢿⣿⣿⣿⣿⣿⣿⠿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣏⡁⣸⣗⢙⣿⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠋⢠⣾⣿⣿⣿⣿⣿⣿⣿⣿⠁⢠⠏⣰⣿⣿⣿⠁⢀⣿⡿⢋⣴⣿⡿⠁⢰⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⢿⣿⡷⠆⣣⣄⡁⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠃⢠⣿⣿⣿⣿⣭⣭⠙⣿⣿⡏⠀⠎⣴⣿⣿⣿⡿⠀⣼⠟⣠⣾⣿⣿⠁⢠⣿⣿⣿⣿⣿⡟⠉⢻⣿⣿⡿⠉⢹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣾⡟⠀⢊⣁⠄⠉⣀⣙⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⢸⣿⣿⣿⣿⣿⣿⠁⣾⡿⠀⢀⣾⣿⣿⣿⣿⠃⢠⠃⣴⣿⣿⣿⠃⢀⣿⡿⠋⢁⡌⢹⠃⠐⠛⠛⣿⠇⠀⠚⠛⣿⡿⠉⢉⣽⡟⠉⣩⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⡿⣿⡿⠟⠑⠙⢉⣄⣤⡉⠻⢿⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣆⠘⢿⣿⣿⣿⡿⢃⣴⣿⠃⢂⣾⣿⣿⣿⣿⡏⡀⢁⣾⣿⣿⣿⠇⠀⣾⡟⠀⡴⠟⣠⠏⣼⡟⠀⣼⡟⣰⡿⠁⣰⡿⠁⢠⣿⡟⠀⣰⣿⡟⢿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⡿⠂⣰⣾⠟⠽⠙⠷⠢⠙⠻⣿⣷⣾⣿⣿⣿⣿⣿⣿⣿⣷⣤⣉⣉⣩⣴⣿⣿⠃⠀⣾⣿⣿⣿⣿⡟⡐⢡⣾⣿⣿⣿⡟⠠⢸⣿⠀⢰⣶⣿⠏⣰⡟⠀⣸⡟⣰⡿⠀⢰⡿⠁⢠⣿⠏⠀⢠⣿⡿⢡⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣾⠟⠁⠊⠁⠁⣠⢦⣠⣀⠲⢶⣿⣿⣹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠃⠀⣼⣿⣿⣿⣿⡟⠀⢠⣿⣿⣿⣿⣿⣇⠀⣿⣿⣄⡘⢛⣡⣴⣿⣧⡀⣋⣴⣿⣧⡀⢋⣴⣄⣘⣡⡞⠀⠾⢋⣴⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣏⣿⣿⣻⠟⠉⠁⠒⢉⣠⣄⠉⢏⠣⠌⠙⠻⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⣤⣼⣿⣿⣿⣿⣿⣷⣶⣿⣿⣿⣿⣿⣿⣿⣦⡈⠻⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠟⠛⣉⡉⠀⣴⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⡿⠟⠋⠀⠴⢋⠕⣱⠻⢦⣄⣀⠐⠶⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⢡⣴⣿⡿⠋⢀⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣽⣿⡿⠶⢒⡀⢤⠤⠊⣱⣿⣦⡁⠉⠙⠒⠈⠙⣯⣿⣿⣿⡇⢸⣿⣿⠟⠛⠉⣉⣉⣉⣉⠙⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡀⠿⠟⠋⣀⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⡷⠦⠀⢀⣀⣠⠴⡺⢻⢻⢯⠻⡛⠶⣈⡙⠻⠿⣟⣿⣿⡇⠸⠋⠀⣠⣶⣿⣿⣿⣿⣿⣿⣦⠀⢻⣿⣿⠿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣯⣁⣀⡂⠈⢉⣉⢀⠬⠒⠁⠀⡈⠓⠬⡀⣀⣀⠐⠶⠾⣿⣿⠗⠀⢠⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⠇⢸⣿⠃⢀⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠋⡠⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣶⣶⣶⣤⣤⣶⡿⠀⡀⢿⣦⣴⣬⣀⣀⣰⣶⣶⣿⠏⠀⣠⠘⢿⣿⣿⣿⣿⣿⣿⣿⣿⠟⢀⡾⠁⢠⣾⣿⣿⡟⠛⣿⣿⣿⡿⠋⠛⣿⣿⣿⣿⣿⣿⡿⠁⣰⢃⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣟⣛⣉⣁⡀⢐⣈⣉⣛⣻⣿⣿⣿⣿⣿⡿⠀⢠⣿⣷⣤⣉⠛⠛⠿⠿⠛⢋⣡⣴⡿⠃⢠⣿⡿⠿⣿⠃⡀⠿⠿⣿⠓⠤⣾⣿⠋⢠⣤⣿⡭⠁⡠⢁⣬⡽⠻⣿⠿⠿⣿⣿⠿⢻⣿⣿⠟⠛⢛⠛⢿⣿⠟⢡⣤⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠀⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠃⠠⢋⠁⠀⣼⡏⣰⡶⠀⣰⡟⠀⣰⣿⠇⡄⠸⣿⣿⠃⠀⣡⣿⠏⠀⠜⡁⠀⡠⢋⠁⢀⣾⠟⠁⣠⣾⠋⠁⣼⠏⡀⠘⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠃⢀⣴⠏⠀⣼⡟⣰⡟⠁⣰⡿⠁⢰⡿⠋⢼⡇⠀⢻⠏⠀⣾⣿⠏⠀⣠⡾⠀⢀⣴⠃⢀⣾⠏⠀⣴⣿⠃⠀⣼⠏⢰⣿⠀⢹⡟⣸");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⡀⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠁⢀⣾⣿⠀⠸⠋⣰⣿⡇⠀⠟⣁⠀⠿⠃⠠⡘⠃⠠⢋⡄⠀⠿⠋⠀⣼⣿⠁⢠⣿⣇⠀⠼⢋⡄⠸⠟⣡⠀⠸⢃⠀⡘⠏⠠⠊⣰⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣤⡈⠛⠻⠿⠿⠟⠛⢋⣡⣶⣶⣿⣿⣿⣷⣶⣾⣿⣿⣿⣶⣾⣿⣷⣶⣿⣷⣶⣶⣶⣿⣿⣷⣶⣶⣾⣿⣿⣷⣾⣿⣿⣷⣶⣿⣿⣶⣾⣿⣷⣶⣿⣷⣶⣶⣶⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣶⣶⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠿⢛⣛⣩⣭⣭⣭⠍⠛⠛⠛⠿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⢟⣩⣴⣶⣿⣿⣿⣿⣿⣿⣿⣧⡀⠀⠀⠙⢷⣶⣬⣝⡛⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⢋⣥⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣄⡀⠀⠙⢿⣿⣿⣿⣶⣭⡛⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⢟⣉⣬⣭⣶⣶⣶⣶⣦⣤⣬⣭⣉⣙⠻⠿⣿⣿⣿⣿⣿⣿⣦⡀⠀⠙⠿⠿⠛⠁⠀⠈⢻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⣥⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣶⣬⣙⠛⠉⠙⠛⠳⡀⠀⠀⠀⠀⠀⠀⠀⠈⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢇⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⣼⣿⣿⠟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣗⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⣿⡿⣡⠀⠹⠛⠋⢘⣋⣉⣉⣉⣛⡛⠛⠿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡆⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⣯⣶⠋⢀⣶⡾⠿⠿⣿⣿⣿⣿⣿⣿⠟⠿⠶⠦⠬⠙⠛⠿⢿⣿⣟⠻⢿⣿⣿⣿⣿⡆⠀⠀⠀⠀⠀⠀⠈⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⢻⡏⠆⠋⠀⠀⢀⢀⣿⣿⣿⣿⣿⣧⣀⣀⠀⠀⠀⠀⠠⣆⠀⠈⠉⠛⠷⣌⠛⠿⣿⠇⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⡑⢠⠻⠋⠉⣉⡉⠻⢿⣿⣿⡿⠟⠉⠉⣒⣈⠙⢲⣄⢸⣷⡀⠀⠸⢧⡈⠳⠼⠟⡀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⡎⠀⢰⡏⠁⠈⢢⠀⣿⣯⠀⠀⣰⡟⠀⠈⢳⡄⠛⢘⡋⠃⠀⠀⢠⠑⠀⠀⠀⠙⡀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⡰⣆⠙⠣⢀⡀⠘⡠⠿⢿⣿⣦⠙⠣⢀⣀⡘⠃⣠⣿⣿⣦⡀⠀⠀⠀⠀⠀⠀⠀⢣⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢡⣿⣿⣿⣷⡆⡔⣿⣿⣿⣧⠠⡝⢿⣷⣾⣷⣿⣿⣿⣿⣿⣿⣿⡆⣄⢃⠀⠀⠀⠀⣿⣆⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⣿⣿⣿⣿⣿⠍⢣⣿⣿⣿⡡⠀⠁⠀⣹⣿⣿⣿⣿⣿⣿⣿⣿⢿⠃⠃⠐⡄⠀⠀⣸⣿⣿⡆⠀⠀⠀⠀⢹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠛⣿⡿⢃⣿⡿⢟⣭⡶⢻⡆⡙⢛⠋⠀⣀⣴⣶⢦⠉⡻⢿⣿⣿⣿⡏⠀⠀⠀⠀⠀⠜⠀⣾⣿⣿⣿⣿⡄⢀⣴⣶⣴⣮⣍⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠰⣬⣴⣿⣬⡶⢟⣡⡾⣫⡼⠇⠈⠻⣿⣿⣿⣿⣷⣕⢽⣶⣭⣙⡛⠁⠴⣊⣂⠄⠈⢦⠣⠙⣿⣿⣿⠃⢠⣿⣿⣿⣿⣿⣿⣷⠘⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡷⢨⣍⡉⠶⠾⠿⠷⠚⠉⠀⠀⣀⣀⠄⢉⠛⠿⠟⣿⣿⣮⣭⠙⠛⠛⠋⠘⠁⠀⠳⣌⢣⠀⠘⣿⣧⠀⢘⣿⣿⣿⣿⣿⣿⣿⠁⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢣⡟⣼⡿⣱⠖⣴⢂⡶⣺⣿⣷⡖⣶⣾⢿⣿⣷⣶⣤⣤⣤⣀⡀⠀⠀⠀⠀⠀⢰⣆⠉⢆⠃⠀⢹⣿⡌⠘⢿⡿⢿⣿⣿⣿⠟⣠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⡜⣼⡟⣰⠋⡼⣡⡟⣱⣿⣿⣿⢇⣿⣿⡸⣿⡟⣿⣿⡿⣿⣿⣿⣷⡰⣤⡖⣾⡏⣿⡆⠀⠀⠀⢘⣿⣿⣶⣄⣓⣓⣛⣁⣥⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⣰⡟⡼⢃⡜⢡⢞⣼⢣⣿⣿⡟⣸⣿⣿⡇⣿⣿⡸⣿⣧⢹⣿⣿⣿⣷⠹⣿⠸⣷⢹⡿⠀⠀⢀⠀⣍⠻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⢛⢁⣿⢸⢁⡞⢠⢏⣾⠃⣾⣿⣿⢇⣿⣿⣿⠁⣿⣿⡇⢻⣿⣇⢻⣿⡘⣿⣇⢻⡇⣿⠘⡇⠀⠀⠀⠀⠀⠘⠷⢶⣭⣛⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⣿⣿⡿⢛⣥⣾⣿⢸⠀⡇⡼⡀⡟⣼⣧⣷⣿⣿⣿⢸⣿⣿⡏⡄⣿⣿⡇⣼⣿⣿⢸⣿⣧⢹⣿⠸⠇⡿⢰⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠛⢶⣭⣛⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⣿⡿⠋⣰⣿⣿⣿⣿⡆⣰⠀⣇⢣⣡⣿⣿⣿⣿⣿⡇⣿⣿⡟⣰⢃⣿⣿⡇⣿⣿⣿⢸⣿⣿⢸⡿⢠⢠⠃⡞⠀⠈⡰⠂⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠛⠹⢦⣝⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⣿⢏⠄⣼⣿⣿⣿⣿⣿⣷⢹⡀⡿⡼⣿⣿⣿⣿⣿⣿⡇⣿⡟⣰⠏⣼⣿⡿⢸⢸⣿⠇⣾⢹⡏⣼⠇⠀⣆⡼⠀⠀⠀⠀⠀⣠⣾⣿⣿⣿⣷⣶⣦⣄⠀⠀⠀⠀⠙⠿⠌⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⢋⡞⣸⣿⣿⣿⣿⣿⣿⣿⣧⢳⢡⢃⢿⣿⣿⣿⣿⣿⡇⡿⣱⡟⣸⣿⡿⢡⠇⣾⠏⣼⠃⡾⢡⠏⣐⡿⠋⠀⠀⠀⠀⣠⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠀⠀⠀⠀⣠⠶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⢃⣾⢁⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⠢⡹⡌⢿⣿⣿⣿⣿⠇⢡⡟⣰⣿⡟⣡⠏⡼⢋⡼⢁⠜⣡⠋⠘⠁⠀⠀⠀⠀⢀⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠃⠀⠀⢀⡜⠁⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⡏⣼⡏⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣧⣴⠌⠈⠻⣿⣿⡽⣾⠸⢡⣿⢏⠔⡡⢊⡴⣋⡴⠟⠘⠁⠀⠀⠀⠀⠀⠀⢠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠇⠀⠀⢠⠋⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣡⣿⡇⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠘⢢⠰⠈⢕⠹⠇⠸⠃⠀⠪⠐⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠉⠀⠀⣰⠃⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⣿⠂⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⣳⣆⠈⣿⣿⠆⠁⠀⠀⠈⢀⠀⠈⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠗⠀⠀⠀⢰⠁⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("⣿⡟⠀⣿⣿⣮⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣠⣿⣿⣿⡂⠀⣠⣶⣶⣦⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢼⣿⣿⣿⣿⣿⣿⣿⣿⠟⠈⠀⠀⠀⢀⠃⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");
        $display("\n------------------------------------------------------------\n");
        $display("Congratulations! All patterns passed!");
        $display("-----------------------------PASS---------------------------\n");
    end else begin
        $display("\n------------------------------------------------------------\n");
        $display("FAIL! %d patterns failed.", total_err_pat);
        $display("------------------------------------------------------------\n");
    end
    $finish;
end

task load_input(input integer idx, input integer num_g);
    reg [8*2-1:0] idx_str;
    integer g_idx;
    reg [31:0] pos_x, pos_y, pos_z;
    reg [31:0] scale_x, scale_y, scale_z;
    reg [31:0] rot_r, rot_i, rot_j, rot_k;
    reg [31:0] opacity;
    reg [31:0] col_r, col_g, col_b;
    reg [7:0] idx_d2, idx_d1, idx_d0;
    begin
        // Calculate digits
        idx_d2 = (idx / 100) + 48;
        idx_d1 = ((idx / 10) % 10) + 48;
        idx_d0 = (idx % 10) + 48;
        
        if (idx < 100) begin
            // 2-digit format: "xx.dat"
            input_gauss_file = "golden_128/input/input_gaussian_00.dat";
            input_gauss_file[4*8 +: 2*8] = {idx_d1, idx_d0}; 
            
            input_view_file = "golden_128/input/input_view_00.dat";
            input_view_file[4*8 +: 2*8] = {idx_d1, idx_d0};
        end else begin
            // 3-digit format: "xxx.dat"
            input_gauss_file = "golden_128/input/input_gaussian_000.dat";
            input_gauss_file[4*8 +: 3*8] = {idx_d2, idx_d1, idx_d0};
            
            input_view_file = "golden_128/input/input_view_000.dat";
            input_view_file[4*8 +: 3*8] = {idx_d2, idx_d1, idx_d0};
        end
        
        $readmemh(input_gauss_file, input_gauss_mem);
        $readmemh(input_view_file, input_view_mem);
        
        // Load View Matrix (16 elements * 16 bits = 256 bits)
        // Slice from 256-bit register and write to 64-bit SRAM (4 words, 4 elements/word)
        for (i=0; i<4; i=i+1) begin
            sram_view_u.load_param(i, {
                input_view_mem[0][255 - 16*(4*i+3) -: 16], // MSB of Word (Elem 3)
                input_view_mem[0][255 - 16*(4*i+2) -: 16],
                input_view_mem[0][255 - 16*(4*i+1) -: 16],
                input_view_mem[0][255 - 16*(4*i+0) -: 16]  // LSB of Word (Elem 0)
            });
        end
        
        // Load into SRAM Models with Packing
        // Input file has 14 params per Gaussian (14 * 16 = 224 bits)
        // Pack into 4 words of 64-bit SRAM
        
        for(g_idx=0; g_idx<num_g; g_idx=g_idx+1) begin 
             // Word 0: {Reserved, Pos Z, Pos Y, Pos X}
             sram_input_u.load_param(g_idx*4 + 0, {
                 16'd0,                             // Reserved
                 input_gauss_mem[g_idx][191 -: 16], // Param 2 (PosZ)
                 input_gauss_mem[g_idx][207 -: 16], // Param 1 (PosY)
                 input_gauss_mem[g_idx][223 -: 16]  // Param 0 (PosX)
             });
             
             // Word 1: {Reserved, Scale Z, Scale Y, Scale X}
             sram_input_u.load_param(g_idx*4 + 1, {
                 16'd0,                             // Reserved
                 input_gauss_mem[g_idx][143 -: 16], // Param 5 (ScaleZ)
                 input_gauss_mem[g_idx][159 -: 16], // Param 4 (ScaleY)
                 input_gauss_mem[g_idx][175 -: 16]  // Param 3 (ScaleX)
             });
             
             // Word 2: {Rot K, Rot J, Rot I, Rot R}
             sram_input_u.load_param(g_idx*4 + 2, {
                 input_gauss_mem[g_idx][79  -: 16], // Param 9 (RotK)
                 input_gauss_mem[g_idx][95  -: 16], // Param 8 (RotJ)
                 input_gauss_mem[g_idx][111 -: 16], // Param 7 (RotI)
                 input_gauss_mem[g_idx][127 -: 16]  // Param 6 (RotR)
             });
             
             // Word 3: {Color B, Color G, Color R, Opacity}
             sram_input_u.load_param(g_idx*4 + 3, {
                 input_gauss_mem[g_idx][15  -: 16], // Param 13 (ColB)
                 input_gauss_mem[g_idx][31  -: 16], // Param 12 (ColG)
                 input_gauss_mem[g_idx][47  -: 16], // Param 11 (ColR)
                 input_gauss_mem[g_idx][63  -: 16]  // Param 10 (Opacity)
             });
        end
    end
endtask

// Golden Memories
reg [143:0] golden_rot_mem [0:1024-1]; // 9 * 16-bit = 144 bits
reg [143:0] golden_cov3d_mem [0:1024-1]; // 9 * 16-bit = 144 bits
reg [127:0] golden_cam_pos_mem [0:1024-1];
reg [95:0] golden_jacobian_mem [0:1024-1];
reg [63:0] golden_cov2d_mem [0:1024-1];
reg [47:0] golden_conic_mem [0:1024-1];
reg [47:0] golden_sort_mem [0:1024-1];
// reg [47:0] golden_sort_mem [0:1024-1];
reg [95:0] golden_bbox_mem [0:1024-1];
reg [127:0] golden_raster_param_mem [0:1024-1];


task load_golden(input integer idx, input integer stage);
    reg [7:0] idx_d2, idx_d1, idx_d0;
    begin
        idx_d2 = (idx / 100) + 48;
        idx_d1 = ((idx / 10) % 10) + 48;
        idx_d0 = (idx % 10) + 48;
        
        if (stage == STAGE1_ROT) begin
            if (idx < 100) begin
                golden_rot_file = "golden_128/stage1_rot/golden_rot_mat_00.dat";
                golden_rot_file[4*8 +: 2*8] = {idx_d1, idx_d0};
            end else begin
                golden_rot_file = "golden_128/stage1_rot/golden_rot_mat_000.dat";
                golden_rot_file[4*8 +: 3*8] = {idx_d2, idx_d1, idx_d0};
            end
            $readmemh(golden_rot_file, golden_rot_mem);
        end
        else if (stage == STAGE2_SCALE) begin
            if (idx < 100) begin
                golden_scale_file = "golden_128/stage2_cov3d/golden_cov3d_00.dat";
                golden_scale_file[4*8 +: 2*8] = {idx_d1, idx_d0};
            end else begin
                golden_scale_file = "golden_128/stage2_cov3d/golden_cov3d_000.dat";
                golden_scale_file[4*8 +: 3*8] = {idx_d2, idx_d1, idx_d0};
            end
            $readmemh(golden_scale_file, golden_cov3d_mem);
        end
        else if (stage == STAGE3_CAM_POS) begin
            if (idx < 100) begin
                golden_cam_pos_file = "golden_128/stage3_cam_pos/golden_cam_pos_00.dat";
                golden_cam_pos_file[4*8 +: 2*8] = {idx_d1, idx_d0};
            end else begin
                 golden_cam_pos_file = "golden_128/stage3_cam_pos/golden_cam_pos_000.dat";
                 golden_cam_pos_file[4*8 +: 3*8] = {idx_d2, idx_d1, idx_d0};
            end
            $readmemh(golden_cam_pos_file, golden_cam_pos_mem);
        end
        else if (stage == STAGE4_JACOBIAN) begin
            if (idx < 100) begin
                golden_jacobian_file = "golden_128/stage4_jacobian/golden_jacobian_00.dat";
                golden_jacobian_file[4*8 +: 2*8] = {idx_d1, idx_d0};
            end else begin
                golden_jacobian_file = "golden_128/stage4_jacobian/golden_jacobian_000.dat";
                golden_jacobian_file[4*8 +: 3*8] = {idx_d2, idx_d1, idx_d0};
            end
            $readmemh(golden_jacobian_file, golden_jacobian_mem);
        end
        else if (stage == STAGE5_COV2D) begin
            if (idx < 100) begin
                golden_cov2d_file = "golden_128/stage5_cov2d/golden_cov2d_00.dat";
                golden_cov2d_file[4*8 +: 2*8] = {idx_d1, idx_d0};
            end else begin
                golden_cov2d_file = "golden_128/stage5_cov2d/golden_cov2d_000.dat";
                golden_cov2d_file[4*8 +: 3*8] = {idx_d2, idx_d1, idx_d0};
            end
            $readmemh(golden_cov2d_file, golden_cov2d_mem);
        end
        else if (stage == STAGE6_CONIC) begin
            if (idx < 100) begin
                golden_conic_file = "golden_128/stage6_conic/golden_conic_00.dat";
                golden_conic_file[4*8 +: 2*8] = {idx_d1, idx_d0};
            end else begin
                golden_conic_file = "golden_128/stage6_conic/golden_conic_000.dat";
                golden_conic_file[4*8 +: 3*8] = {idx_d2, idx_d1, idx_d0};
            end
            $readmemh(golden_conic_file, golden_conic_mem);
        end
        else if (stage == STAGE7_SORT) begin
            if (idx < 100) begin
                golden_sort_file = "golden_128/stage7_sort/golden_sort_00.dat";
                golden_sort_file[4*8 +: 2*8] = {idx_d1, idx_d0};
            end else begin
                golden_sort_file = "golden_128/stage7_sort/golden_sort_000.dat";
                golden_sort_file[4*8 +: 3*8] = {idx_d2, idx_d1, idx_d0};
            end
            $readmemh(golden_sort_file, golden_sort_mem);
        end
        else if (stage == STAGE8_RENDER) begin
            if (idx < 100) begin
                golden_image_file = "golden_128/stage8_render/golden_image_00.dat";
                golden_image_file[4*8 +: 2*8] = {idx_d1, idx_d0};
            end else begin
                golden_image_file = "golden_128/stage8_render/golden_image_000.dat";
                golden_image_file[4*8 +: 3*8] = {idx_d2, idx_d1, idx_d0};
            end
            $readmemh(golden_image_file, golden_img_mem);
        end
        else if (stage == STAGE_BBOX) begin
            if (idx < 100) begin
                golden_bbox_file = "golden_128/stage_bbox/golden_bbox_00.dat";
                golden_bbox_file[4*8 +: 2*8] = {idx_d1, idx_d0};
            end else begin
                golden_bbox_file = "golden_128/stage_bbox/golden_bbox_000.dat";
                golden_bbox_file[4*8 +: 3*8] = {idx_d2, idx_d1, idx_d0};
            end
            $readmemh(golden_bbox_file, golden_bbox_mem);
        end
        else if (stage == STAGE_RASTER_PARAM) begin
           if (idx < 100) begin
                golden_raster_param_file = "golden_128/stage_raster_param/golden_raster_param_00.dat";
                golden_raster_param_file[4*8 +: 2*8] = {idx_d1, idx_d0};
            end else begin
                golden_raster_param_file = "golden_128/stage_raster_param/golden_raster_param_000.dat";
                golden_raster_param_file[4*8 +: 3*8] = {idx_d2, idx_d1, idx_d0};
            end
            $readmemh(golden_raster_param_file, golden_raster_param_mem);
        end
    end
endtask

// Verification Tasks
// Verification Tasks
// Note: These tasks assume the DUT exposes internal signals/arrays for verification.
// You should assign 'uut.debug_stageX_data' to your internal signals.

task verify_stage1; // Rot
    integer k;
    reg [143:0] gold_val;
    reg [143:0] dut_val;
    begin
        for(k=0; k<num_gaussians; k=k+1) begin
             gold_val = golden_rot_mem[k];
             // Hierarchical reference to DUT internal signal
             // Ensure your DUT has: wire [127:0] debug_stage1_data [0:1023];
             //dut_val = uut.debug_stage1_data[k];  
             
             // For compilation, we comment out the actual access:
            dut_val = 144'h0; // Placeholder
             
             if (dut_val !== gold_val) begin
                 // Uncomment when signals are ready
                 $display("Stage 1 Error at idx %d: Expected %h, Got %h", k, gold_val, dut_val);
                 $display("      Matrix Comparison (3x3):");
                 $display("      Row 0: Exp [%h %h %h] | Got [%h %h %h]", 
                     gold_val[143:128], gold_val[127:112], gold_val[111:96],
                     dut_val[143:128], dut_val[127:112], dut_val[111:96]);
                 $display("      Row 1: Exp [%h %h %h] | Got [%h %h %h]", 
                     gold_val[95:80], gold_val[79:64], gold_val[63:48],
                     dut_val[95:80], dut_val[79:64], dut_val[63:48]);
                 $display("      Row 2: Exp [%h %h %h] | Got [%h %h %h]", 
                     gold_val[47:32], gold_val[31:16], gold_val[15:0],
                     dut_val[47:32], dut_val[31:16], dut_val[15:0]);
                 err_cnt = err_cnt + 1;
             end
        end
    end
endtask

task verify_stage2; // Cov3D (was Scale)
    integer k;
    reg [143:0] gold_val;
    reg [143:0] dut_val;
    begin
        for(k=0; k<num_gaussians; k=k+1) begin
             gold_val = golden_cov3d_mem[k];
             //dut_val = uut.debug_stage2_data[k];
             dut_val = 144'h0; 
             if (dut_val !== gold_val) begin
                 // Uncomment when signals are ready
                 $display("Stage 2 (Cov3D) Error at idx %d: Expected %h, Got %h", k, gold_val, dut_val);
                 $display("      Matrix Comparison (3x3):");
                 $display("      Row 0: Exp [%h %h %h] | Got [%h %h %h]", 
                     gold_val[143:128], gold_val[127:112], gold_val[111:96],
                     dut_val[143:128], dut_val[127:112], dut_val[111:96]);
                 $display("      Row 1: Exp [%h %h %h] | Got [%h %h %h]", 
                     gold_val[95:80], gold_val[79:64], gold_val[63:48],
                     dut_val[95:80], dut_val[79:64], dut_val[63:48]);
                 $display("      Row 2: Exp [%h %h %h] | Got [%h %h %h]", 
                     gold_val[47:32], gold_val[31:16], gold_val[15:0],
                     dut_val[47:32], dut_val[31:16], dut_val[15:0]);
                 err_cnt = err_cnt + 1;
             end
        end
    end
endtask

task verify_stage3; // Cam Pos
    integer k;
    reg [127:0] gold_val;
    reg [127:0] dut_val;
    begin
        for(k=0; k<num_gaussians; k=k+1) begin
             gold_val = golden_cam_pos_mem[k];
             //dut_val = uut.debug_stage3_data[k];
             dut_val = 0;
             // Compare only the relevant 48 bits (3 * 16)
             if (dut_val[47:0] !== gold_val[47:0]) begin
                 $display("Stage 3 (Cam Pos) Error at idx %d: Expected %h, Got %h", k, gold_val[47:0], dut_val[47:0]);
                 $display("      Vector Comparison (3x16):");
                 $display("      Exp [X:%h Y:%h Z:%h] | Got [X:%h Y:%h Z:%h]", 
                     gold_val[47:32], gold_val[31:16], gold_val[15:0],
                     dut_val[47:32], dut_val[31:16], dut_val[15:0]);
                 err_cnt = err_cnt + 1;
             end
        end
    end
endtask

task verify_stage4; // Jacobian
    integer k;
    reg [95:0] gold_val;
    reg [95:0] dut_val;
    begin
        for(k=0; k<num_gaussians; k=k+1) begin
             gold_val = golden_jacobian_mem[k];
             //dut_val = uut.debug_stage4_data[k];
             dut_val = 96'h0;
             if (dut_val !== gold_val) begin
                 $display("Stage 4 (Jacobian) Error at idx %d: Expected %h, Got %h", k, gold_val, dut_val);
                 $display("      Matrix Comparison (2x3):");
                 $display("      Row 0: Exp [%h %h %h] | Got [%h %h %h]", 
                     gold_val[95:80], gold_val[79:64], gold_val[63:48],
                     dut_val[95:80], dut_val[79:64], dut_val[63:48]);
                 $display("      Row 1: Exp [%h %h %h] | Got [%h %h %h]", 
                     gold_val[47:32], gold_val[31:16], gold_val[15:0],
                     dut_val[47:32], dut_val[31:16], dut_val[15:0]);
                 err_cnt = err_cnt + 1;
             end
        end
    end
endtask

task verify_stage5; // Cov2D
    integer k;
    reg [63:0] gold_val;
    reg [63:0] dut_val;
    begin
        for(k=0; k<num_gaussians; k=k+1) begin
             gold_val = golden_cov2d_mem[k];
             //dut_val = uut.debug_stage5_data[k];
             dut_val = 64'h0;
             if (dut_val !== gold_val) begin
                 $display("Stage 5 (Cov2D) Error at idx %d: Expected %h, Got %h", k, gold_val, dut_val);
                 $display("      Matrix Comparison (2x2):");
                 $display("      Row 0: Exp [%h %h] | Got [%h %h]", 
                     gold_val[63:48], gold_val[47:32], 
                     dut_val[63:48], dut_val[47:32]);
                 $display("      Row 1: Exp [%h %h] | Got [%h %h]", 
                     gold_val[31:16], gold_val[15:0], 
                     dut_val[31:16], dut_val[15:0]);
                 err_cnt = err_cnt + 1;
             end
        end
    end
endtask

task verify_stage6; // Conic
    integer k;
    reg [47:0] gold_val;
    reg [47:0] dut_val;
    begin
        for(k=0; k<num_gaussians; k=k+1) begin
             gold_val = golden_conic_mem[k];
             // dut_val = uut.debug_stage6_data[k];
             dut_val = 48'h0;
             if (dut_val !== gold_val) begin
                 $display("Stage 6 (Conic) Error at idx %d: Expected %h, Got %h", k, gold_val, dut_val);
                 $display("      Vector Comparison (3x16):");
                 $display("      Exp [A:%h B:%h C:%h] | Got [A:%h B:%h C:%h]", 
                     gold_val[47:32], gold_val[31:16], gold_val[15:0],
                     dut_val[47:32], dut_val[31:16], dut_val[15:0]);
                 err_cnt = err_cnt + 1;
             end
        end
    end
endtask

task verify_stage7; // Sort
    integer k;
    reg [31:0] gold_val;
    reg [31:0] dut_val;
    begin
        for(k=0; k<num_gaussians; k=k+1) begin
             gold_val = golden_sort_mem[k];
             // dut_val = sram_sort_u.mem[k];
             dut_val = 48'h0;
             if (dut_val !== gold_val) begin
                 $display("Stage 7 (Sort) Error at idx %d: Expected %h, Got %h", k, gold_val, dut_val);
                 $display("      Exp [ID:%h Depth:%h] | Got [ID:%h Depth:%h]",
                     gold_val[31:16], gold_val[15:0],
                     dut_val[31:16], dut_val[15:0]);
                 err_cnt = err_cnt + 1;
             end
        end
    end
endtask

task verify_render;
    integer j;
    reg [63:0] dut_val;
    reg [63:0] gold_val;
    begin
        for(j=0; j<16384; j=j+1) begin
            dut_val = sram_img_u.mem[j]; 
            gold_val = golden_img_mem[j];
            
            if (dut_val !== gold_val) begin
                $display("Error at pixel %d: Expected %h (T: %h R: %h G: %h B: %h), Got %h (T: %h R: %h G: %h B: %h)", 
                    j, gold_val, 
                    gold_val[63:48], gold_val[47:32], gold_val[31:16], gold_val[15:0],
                    dut_val, 
                    dut_val[63:48], dut_val[47:32], dut_val[31:16], dut_val[15:0]);
                err_cnt = err_cnt + 1;
            end
        end
    end
endtask

task verify_bbox;
    integer k;
    reg [95:0] gold_val;
    reg [95:0] dut_val;
    begin
        for(k=0; k<num_gaussians; k=k+1) begin
             gold_val = golden_bbox_mem[k];
             // dut_val = uut.bbox_info[k]; // Assuming exposed
             dut_val = 96'h0;
             
             if (dut_val !== gold_val) begin
                 $display("BBox Error at idx %d: Expected %h, Got %h", k, gold_val, dut_val);
                 $display("      Screen Pos: Exp [X:%h Y:%h] | Got [X:%h Y:%h]", 
                     gold_val[95:80], gold_val[79:64], dut_val[95:80], dut_val[79:64]);
                 $display("      BBox: Exp [minX:%h maxX:%h minY:%h maxY:%h] | Got [minX:%h maxX:%h minY:%h maxY:%h]",
                     gold_val[63:48], gold_val[47:32], gold_val[31:16], gold_val[15:0],
                     dut_val[63:48], dut_val[47:32], dut_val[31:16], dut_val[15:0]);
                 err_cnt = err_cnt + 1;
             end
        end
    end
endtask


task verify_raster_param;
    integer k;
    reg [127:0] gold_val;
    reg [127:0] dut_val;
    begin
        for(k=0; k<num_gaussians; k=k+1) begin
             gold_val = golden_raster_param_mem[k];
             dut_val = sram_raster_u.mem[k]; // Check SRAM directly
             
             if (dut_val !== gold_val) begin
                 $display("Raster Param Error at idx %d:", k);
                 $display("      Expected: %h", gold_val);
                 $display("      Got     : %h", dut_val);
                 
                 // Breakdown
                 $display("      Exp [Ca:%h Cb:%h Cc:%h Cx:%h Cy:%h Rx:%h Ry:%h]",
                    gold_val[15:0], gold_val[31:16], gold_val[47:32], 
                    gold_val[63:48], gold_val[79:64], 
                    gold_val[95:80], gold_val[111:96]);
                 $display("      Got [Ca:%h Cb:%h Cc:%h Cx:%h Cy:%h Rx:%h Ry:%h]",
                    dut_val[15:0], dut_val[31:16], dut_val[47:32], 
                    dut_val[63:48], dut_val[79:64], 
                    dut_val[95:80], dut_val[111:96]);
                    
                 err_cnt = err_cnt + 1;
             end
        end
    end
endtask

endmodule
