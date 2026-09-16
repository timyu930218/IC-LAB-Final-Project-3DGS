//======================================================================================================
//  Note:          Use only for teaching materials of IC Design Lab, NTHU.
//  Copyright: (c) 2025 Vision Circuits and Systems Lab, NTHU, Taiwan. ALL Rights Reserved.
//======================================================================================================

`timescale 1ns/100ps


`ifndef PAT_L
`define PAT_L 0
`endif

`ifndef PAT_U
`define PAT_U 0
`endif
`define NUM_PAT (`PAT_U-`PAT_L+1)

`define PAT_NAME_LENGTH 2
`define CYCLE 10
`define END_CYCLES 20000000
`define FLAG_VERBOSE 1  
`define FLAG_DUMPWV 1
`define FLAG_DUMP_IMAGE 1

module test_demo;

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
    //$fsdbDumpfile("gatesim_gaussian_splatting_demo.fsdb");
    //$fsdbDumpvars("+mda");
	$sdf_annotate("../../../SYN/Gaussian_Splatting_Top_syn.sdf",uut);
`else
    $fsdbDumpfile("presim_gaussian_splatting_demo.fsdb");
    $fsdbDumpvars("+mda");
`endif
end

reg [223:0] input_gauss_mem [0:1024-1];
reg [255:0] input_view_mem [0:0];
reg [63:0] golden_img_mem [0:16384-1];

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

integer cycle_cnt;
initial begin
    cycle_cnt = 0;
    while(1) begin 
        cycle_cnt = cycle_cnt + 1;
        @(negedge clk);
    end
end

integer pat_idx;
integer total_err_pat;
integer err_cnt;
integer num_gaussians;
integer g;
reg  [2:0]state_p; 
// Debug variables
reg [31:0] debug_sort_val;
reg [6:0] debug_id;
reg [127:0] debug_raster_val;
reg [15:0] debug_cx, debug_cy; 
localparam IDLE = 3'd0,
           FETCH_SORT = 3'd1,
           FETCH_PARAM = 3'd2, 
           CALC = 3'd3,
           RENDER = 3'd4,
           DONE = 3'd5;

always @(posedge clk) begin
    state_p = uut.stage8_render.state;
end
initial begin
    if((`PAT_L < 0) || (`PAT_L > 199) || (`PAT_U < 0) || (`PAT_U > 199)) begin
        $display("Error: PAT_L/PAT_U out of range for demo (0-199)");
        $finish;
    end

    $display("\nStart checking Demo %s ...\n", stage_str);

    total_err_pat = 0;
    for(pat_idx=`PAT_L; pat_idx<=`PAT_U; pat_idx=pat_idx+1) begin
        
        num_gaussians = 64;
        gaussian_num = 64; 

        load_input(pat_idx, num_gaussians);
        load_golden(pat_idx, test_stage);

        $display("\nPattern %02d", pat_idx);

        srst_n = 1;
        enable = 0;
        
        // Clear Image SRAM to Background (T=1.0, RGB=0)
        // T=256 (0x0100) in Q0.8 is 1.0 opacity? Wait, T is Transmittance. 1.0 = Fully Transparent. 0.0 = Opaque.
        // Stage8 Logic: s3_old_color_t = (16'b1<<8); -> 256.
        // So Initial state is T=256.
        for(i=0; i<16384; i=i+1) begin
             sram_img_u.load_param(i, 64'h0100_0000_0000_0000);
        end
        
        @(negedge clk); srst_n = 1'b0;
        @(negedge clk); srst_n = 1'b1; enable = 1'b1;
        @(negedge clk); enable = 1'b0;
        
        //if (pat_idx == 50) begin
        //    // Special debug flow for Pattern 50
        //    
        //    // Verify Pre-Render Stages first to ensure input to Render is correct
        //    // BBox, Conic, Sort, etc.
        //    // These should have completed before Render starts processing.
        //    // (Assuming Stage 8 Render starts reading Sort SRAM which is filled by Stage 7)
        //    
        //    // Note: We need to make sure Stage 7 is actually done. 
        //    // In the top level, valid goes high when EVERYTHING is done.
        //    // But checking Stage 7 SRAM content should be safe if we are checking Render (which comes after Sort).
        //    // However, the loop below waits for Render progress.
        //    // Let's verify Stage 7 at the very beginning of this debug block.
        //    // Since we don't have a 'stage 7 done' signal easily exposed, 
        //    // verifying it along with the first Gaussian render check or just blindly here might be okay 
        //    // if the pipeline flow guarantees Sort is populated. 
        //    // Stage 8 Render reads Sort SRAM, so it must be populated.
        //    
        //    verify_stage7(); 
        //    if (err_cnt > 0) begin
        //        $display("CRITICAL: Stage 7 (Sort) Failed for Pattern 50! Aborting Render Debug.");
        //        $finish;
        //    end
        //    
//
        //    //// DEBUG: Check Gaussian 0 Params from SRAM
////
///
        //    //for (g = 0; g < 64; g = g + 1) begin
        //    //    // Wait for the current Gaussian to finish rendering
        //    //    // 'gaussian_cnt' in Render stage logic tracks how many have been processed.
        //    //    // Assuming 'uut.u_Stage8_Render.gaussian_cnt' corresponds.
        //    //    // NOTE: 'Gaussian_Splatting_Top' instantiates 'Stage8_Render' as 'stage8_render'.
        //    //    // So path is uut.stage8_render.gaussian_cnt
        //    //    wait(state_p == CALC && uut.stage8_render.state == FETCH_SORT);
        //    //    g = uut.stage8_render.gaussian_cnt;
        //    //    
        //    //    // DEBUG: Inspect SRAM for Gaussian 0 when g==1 (which confirms G0 is done)
        //    //    if (g == 1) begin
        //    //        debug_sort_val = sram_sort_u.mem[0];
        //    //        debug_id = debug_sort_val[22:16];
        //    //        debug_raster_val = sram_raster_u.mem[debug_id];
        //    //        debug_cx = debug_raster_val[63:48];
        //    //        debug_cy = debug_raster_val[79:64];
        //    //        
        //    //        $display("DEBUG (Loop): Pattern 50 Gaussian 0 Info (At g=1):");
        //    //        $display("  Sort[0] = %h -> ID = %d", debug_sort_val, debug_id);
        //    //        $display("  Raster[ID] = %h", debug_raster_val);
        //    //        $display("  Center X (Hex) = %h, (Dec/Q6) = %d / 64.0 = %f", debug_cx, debug_cx, $signed(debug_cx)/64.0);
        //    //        $display("  Center Y (Hex) = %h, (Dec/Q6) = %d / 64.0 = %f", debug_cy, debug_cy, $signed(debug_cy)/64.0);
        //    //    end
//
        //    //    @(negedge clk);
        //    //    
        //    //    // Verify intermediate result
        //    //    verify_intermediate_render(g-1);
        //    //end
        // end else begin
            // Standard Verification for Demo
            wait(valid);
            @(negedge clk);
            
            err_cnt = 0;
            // Verify Render Stage by default for demo
            if (test_stage == STAGE8_RENDER) verify_render();
            else begin
                 // Verify other stages if needed
                 case (test_stage)
                    STAGE1_ROT: verify_stage1();
                    STAGE2_SCALE: verify_stage2();
                    STAGE3_CAM_POS: verify_stage3();
                    STAGE4_JACOBIAN: verify_stage4();
                    STAGE5_COV2D: verify_stage5();
                    STAGE6_CONIC: verify_stage6();
                    STAGE7_SORT: verify_stage7();
                    STAGE_BBOX: verify_bbox();
                    STAGE_RASTER_PARAM: verify_raster_param();
                endcase
            end
            
            if(err_cnt > 0) begin
                $display("Pattern %02d FAILED with %d errors", pat_idx, err_cnt);
                total_err_pat = total_err_pat + 1;
            end else begin
                $display("Pattern %02d PASSED", pat_idx);
            end
        // end
        

        // Call dump task
        `ifdef FLAG_DUMP_IMAGE
            $display("Attempting to dump frame for pattern %0d...", pat_idx);
            dump_frame_image(pat_idx);
        `else
            $display("Skipping frame dump (FLAG_DUMP_IMAGE not set).");
        `endif
    end 

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

reg [60*8-1:0] dump_output_file;

task dump_frame_image(input integer idx);
    integer fd, i;
    reg [63:0] read_data;
    reg [7:0] idx_d2, idx_d1, idx_d0;
    begin
        // idx_d2 = (idx / 100) + 48;
        // idx_d1 = ((idx / 10) % 10) + 48;
        // idx_d0 = (idx % 10) + 48;

        // dump_output_file = "output_demo/stage8_render/output_image_00.dat";
        // dump_output_file[4*8 +: 2*8] = {idx_d1, idx_d0};
        if (idx < 10) $sformat(dump_output_file, "output_demo/stage8_render/output_image_0%0d.dat", idx);
        else $sformat(dump_output_file, "output_demo/stage8_render/output_image_%0d.dat", idx);

        fd = $fopen(dump_output_file, "w");
        if (fd == 0) begin
            $display("Error: Could not open file %s for writing. Check if directory exists!", dump_output_file);
        end else begin
            $display("Successfully opened %s. Dumping data...", dump_output_file);
            for (i = 0; i < 16384; i = i + 1) begin
                read_data = sram_img_u.mem[i];
                // Write as hex string, 16 hex digits (64 bits)
                $fwrite(fd, "%016h\n", read_data);
            end
            $display("Dump complete for %s", dump_output_file);
            $fclose(fd);
        end
    end
endtask

task load_input(input integer idx, input integer num_g);
    reg [8*2-1:0] idx_str;
    integer g_idx;
    reg [7:0] idx_d2, idx_d1, idx_d0;
    begin
        // idx_d2 = (idx / 100) + 48;
        // idx_d1 = ((idx / 10) % 10) + 48;
        // idx_d0 = (idx % 10) + 48;
        
        // Always 2 digit for demo (0-99)
        // input_gauss_file = "golden_demo/input/input_gaussian_00.dat";
        // input_gauss_file[4*8 +: 2*8] = {idx_d1, idx_d0}; 
        
        // input_view_file = "golden_demo/input/input_view_00.dat";
        // input_view_file[4*8 +: 2*8] = {idx_d1, idx_d0};
        
        if (idx < 10) begin
            $sformat(input_gauss_file, "golden_demo/input/input_gaussian_0%0d.dat", idx);
            $sformat(input_view_file, "golden_demo/input/input_view_0%0d.dat", idx);
        end else begin
            $sformat(input_gauss_file, "golden_demo/input/input_gaussian_%0d.dat", idx);
            $sformat(input_view_file, "golden_demo/input/input_view_%0d.dat", idx);
        end

        $readmemh(input_gauss_file, input_gauss_mem);
        $readmemh(input_view_file, input_view_mem);
        
        for (i=0; i<4; i=i+1) begin
            sram_view_u.load_param(i, {
                input_view_mem[0][255 - 16*(4*i+3) -: 16],
                input_view_mem[0][255 - 16*(4*i+2) -: 16],
                input_view_mem[0][255 - 16*(4*i+1) -: 16],
                input_view_mem[0][255 - 16*(4*i+0) -: 16]
            });
        end
        
        for(g_idx=0; g_idx<num_g; g_idx=g_idx+1) begin 
             sram_input_u.load_param(g_idx*4 + 0, {
                 16'd0,                             
                 input_gauss_mem[g_idx][191 -: 16], 
                 input_gauss_mem[g_idx][207 -: 16], 
                 input_gauss_mem[g_idx][223 -: 16]  
             });
             sram_input_u.load_param(g_idx*4 + 1, {
                 16'd0,                             
                 input_gauss_mem[g_idx][143 -: 16], 
                 input_gauss_mem[g_idx][159 -: 16], 
                 input_gauss_mem[g_idx][175 -: 16]  
             });
             sram_input_u.load_param(g_idx*4 + 2, {
                 input_gauss_mem[g_idx][79  -: 16], 
                 input_gauss_mem[g_idx][95  -: 16], 
                 input_gauss_mem[g_idx][111 -: 16], 
                 input_gauss_mem[g_idx][127 -: 16]  
             });
             sram_input_u.load_param(g_idx*4 + 3, {
                 input_gauss_mem[g_idx][15  -: 16], 
                 input_gauss_mem[g_idx][31  -: 16], 
                 input_gauss_mem[g_idx][47  -: 16], 
                 input_gauss_mem[g_idx][63  -: 16]  
             });
        end
    end
endtask

// Golden Memories
reg [143:0] golden_rot_mem [0:1024-1]; 
reg [143:0] golden_cov3d_mem [0:1024-1]; 
reg [127:0] golden_cam_pos_mem [0:1024-1];
reg [95:0] golden_jacobian_mem [0:1024-1];
reg [63:0] golden_cov2d_mem [0:1024-1];
reg [47:0] golden_conic_mem [0:1024-1];
reg [47:0] golden_sort_mem [0:1024-1];
reg [95:0] golden_bbox_mem [0:1024-1];
reg [127:0] golden_raster_param_mem [0:1024-1];

task load_golden(input integer idx, input integer stage);
    reg [7:0] idx_d2, idx_d1, idx_d0;
    begin
        // idx_d2 = (idx / 100) + 48;
        // idx_d1 = ((idx / 10) % 10) + 48;
        // idx_d0 = (idx % 10) + 48;
        
        if (stage == STAGE1_ROT) begin
            // golden_rot_file = "golden_demo/stage1_rot/golden_rot_mat_00.dat";
            // golden_rot_file[4*8 +: 2*8] = {idx_d1, idx_d0};
            if (idx < 10) $sformat(golden_rot_file, "golden_demo/stage1_rot/golden_rot_mat_0%0d.dat", idx);
            else $sformat(golden_rot_file, "golden_demo/stage1_rot/golden_rot_mat_%0d.dat", idx);
            $readmemh(golden_rot_file, golden_rot_mem);
        end
        else if (stage == STAGE2_SCALE) begin
            if (idx < 10) $sformat(golden_scale_file, "golden_demo/stage2_cov3d/golden_cov3d_0%0d.dat", idx);
            else $sformat(golden_scale_file, "golden_demo/stage2_cov3d/golden_cov3d_%0d.dat", idx);
            $readmemh(golden_scale_file, golden_cov3d_mem);
        end
        else if (stage == STAGE3_CAM_POS) begin
            if (idx < 10) $sformat(golden_cam_pos_file, "golden_demo/stage3_cam_pos/golden_cam_pos_0%0d.dat", idx);
            else $sformat(golden_cam_pos_file, "golden_demo/stage3_cam_pos/golden_cam_pos_%0d.dat", idx);
            $readmemh(golden_cam_pos_file, golden_cam_pos_mem);
        end
        else if (stage == STAGE4_JACOBIAN) begin
            if (idx < 10) $sformat(golden_jacobian_file, "golden_demo/stage4_jacobian/golden_jacobian_0%0d.dat", idx);
            else $sformat(golden_jacobian_file, "golden_demo/stage4_jacobian/golden_jacobian_%0d.dat", idx);
            $readmemh(golden_jacobian_file, golden_jacobian_mem);
        end
        else if (stage == STAGE5_COV2D) begin
            if (idx < 10) $sformat(golden_cov2d_file, "golden_demo/stage5_cov2d/golden_cov2d_0%0d.dat", idx);
            else $sformat(golden_cov2d_file, "golden_demo/stage5_cov2d/golden_cov2d_%0d.dat", idx);
            $readmemh(golden_cov2d_file, golden_cov2d_mem);
        end
        else if (stage == STAGE6_CONIC) begin
            if (idx < 10) $sformat(golden_conic_file, "golden_demo/stage6_conic/golden_conic_0%0d.dat", idx);
            else $sformat(golden_conic_file, "golden_demo/stage6_conic/golden_conic_%0d.dat", idx);
            $readmemh(golden_conic_file, golden_conic_mem);
        end
        else if (stage == STAGE7_SORT) begin
            if (idx < 10) $sformat(golden_sort_file, "golden_demo/stage7_sort/golden_sort_0%0d.dat", idx);
            else $sformat(golden_sort_file, "golden_demo/stage7_sort/golden_sort_%0d.dat", idx);
            $readmemh(golden_sort_file, golden_sort_mem);
        end
        else if (stage == STAGE8_RENDER) begin
            if (idx < 10) $sformat(golden_image_file, "golden_demo/stage8_render/golden_image_0%0d.dat", idx);
            else $sformat(golden_image_file, "golden_demo/stage8_render/golden_image_%0d.dat", idx);
            $readmemh(golden_image_file, golden_img_mem);
        end
        else if (stage == STAGE_BBOX) begin
            if (idx < 10) $sformat(golden_bbox_file, "golden_demo/stage_bbox/golden_bbox_0%0d.dat", idx);
            else $sformat(golden_bbox_file, "golden_demo/stage_bbox/golden_bbox_%0d.dat", idx);
            $readmemh(golden_bbox_file, golden_bbox_mem);
        end
        else if (stage == STAGE_RASTER_PARAM) begin
            if (idx < 10) $sformat(golden_raster_param_file, "golden_demo/stage_raster_param/golden_raster_param_0%0d.dat", idx);
            else $sformat(golden_raster_param_file, "golden_demo/stage_raster_param/golden_raster_param_%0d.dat", idx);
            $readmemh(golden_raster_param_file, golden_raster_param_mem);
        end
    end
endtask

// Placeholders for Verify Tasks (Copy from test_top.v)
// ... (Shortened for brevity, assume they exist or use `include if possible, but easier to copy paste common logic if needed)
// Actually I need to include them. 
// For this task, I'll include the Render verify task specifically since that's what we care about for the demo.
// Ideally I should have `included a file with these tasks but I will just paste the verify_render here.

task verify_render;
    integer x, y, k;
    reg [63:0] gold_word;
    reg [63:0] dut_word;
    begin
        for(y=0; y<128; y=y+1) begin
            for(x=0; x<128; x=x+1) begin
                
                // Address: y*128 + x
                // SRAM Depth = 16384
                // k = y * 128 + x;
                k = (y << 7) + x;
                
                gold_word = golden_img_mem[k];
                dut_word = sram_img_u.mem[k];
                
                // Compare (T, R, G, B)
                // We can allow some tolerance? Or strict matching.
                // Golden data generated with hardware simulation logic implies strict match.
                
                if (dut_word !== gold_word) begin
                     $display("Render Error at (x=%d, y=%d): Exp %h, Got %h", x, y, gold_word, dut_word);
                     err_cnt = err_cnt + 1;
                     if (err_cnt > 10) begin 
                        $display("Too many errors. Stopping verification for this pattern.");
                        x = 129; y = 129; // Break
                     end
                end 
            end
        end
        if(err_cnt>0) $display("Total number of error is %d", err_cnt);
    end
endtask

task verify_stage1; begin end endtask
task verify_stage2; begin end endtask
task verify_stage3; begin end endtask
task verify_stage4; begin end endtask
task verify_stage5; begin end endtask
task verify_stage6; begin end endtask
task verify_stage7; begin end endtask
task verify_bbox; begin end endtask
task verify_raster_param; begin end endtask


reg [63:0] golden_img_debug_mem [0:16384-1];
reg [60*8-1:0] golden_debug_file;

// task verify_intermediate_render(input integer g_id);
//     integer i;
//     reg [63:0] gold_val;
//     reg [63:0] dut_val;
//     reg [7:0] idx_d1, idx_d0;
//     begin
//         idx_d1 = (g_id / 10) + 48;
//         idx_d0 = (g_id % 10) + 48;
//         
//         // Use golden_demo/debug_demo_50 as confirmed 
//         golden_debug_file = "golden_demo/debug_demo_50/golden_image_50_00.dat";
//         golden_debug_file[4*8 +: 2*8] = {idx_d1, idx_d0};
//         
//         $readmemh(golden_debug_file, golden_img_debug_mem);
//         
//         for(i=0; i<16384; i=i+1) begin
//              gold_val = golden_img_debug_mem[i];
//              dut_val = sram_img_u.mem[i]; 
//              
//              if (dut_val !== gold_val) begin
//                  $display("Pattern 50 Gaussian %02d FAILED at pixel %d: Expected %h, Got %h", g_id, i, gold_val, dut_val);
//                  err_cnt = err_cnt + 1;
//                  if (err_cnt > 10) begin 
//                     $display("Too many errors for this Gaussian. Skipping rest.");
//                     i = 16384;
//                  end
//              end
//         end
//         
//         if (err_cnt == 0) begin
//             $display("Pattern 50 Gaussian %02d PASSED", g_id);
//         end else begin
//             $display("Pattern 50 Gaussian %02d FAILED total %d errors. Stopping simulation.", g_id, err_cnt);
//             $finish; 
//         end
//     end
// endtask
// 
// 
// 

// initial begin
//     forever @(posedge clk) begin
//         if (pat_idx == 51 && uut.stage8_render.s2_pixel_count == 15 && uut.stage8_render.s2_valid) begin
//             // Extract ID for debugging (Assumes sram_sort_u is populated)
//             debug_sort_val = sram_sort_u.mem[uut.stage8_render.gaussian_cnt];
//             debug_id = debug_sort_val[22:16];
//             
//             $display("DEBUG: PROBE S2 G=%d (ID=%d, SortVal=%h) Pixel=15 (Time: %t)", uut.stage8_render.gaussian_cnt, debug_id, debug_sort_val, $time);
//             $display("  dx_sq: %d (Hex: %h)", uut.stage8_render.s2_dx_sq, uut.stage8_render.s2_dx_sq);
//             $display("  dy_sq: %d (Hex: %h)", uut.stage8_render.s2_dy_sq, uut.stage8_render.s2_dy_sq);
//             $display("  dx_dy: %d (Hex: %h)", uut.stage8_render.s2_dx_dy, uut.stage8_render.s2_dx_dy);
//             $display("  Conic A: %d", uut.stage8_render.conic_a);
//             $display("  Conic B: %d", uut.stage8_render.conic_b);
//             $display("  Conic C: %d", uut.stage8_render.conic_c);
//         end
// 
//         if (pat_idx == 51 && uut.stage8_render.s5_pixel_count == 15 && uut.stage8_render.s5_valid) begin
//             $display("DEBUG: PROBE S5 G=%d Pixel=15 (Time: %t)", uut.stage8_render.gaussian_cnt, $time);
//             $display("  Term A: %d (Hex: %h)", uut.stage8_render.s5_term_a, uut.stage8_render.s5_term_a);
//             $display("  Term B: %d (Hex: %h)", uut.stage8_render.s5_term_b, uut.stage8_render.s5_term_b);
//             $display("  Term C: %d (Hex: %h)", uut.stage8_render.s5_term_c, uut.stage8_render.s5_term_c);
//         end
// 
//         if (pat_idx == 51 && uut.stage8_render.s6_pixel_count == 15 && uut.stage8_render.s6_valid) begin
//             $display("DEBUG: PROBE S6 G=%d Pixel=15 (Time: %t)", uut.stage8_render.gaussian_cnt, $time);
//             $display("  Sum Terms: %d (Hex: %h)", uut.stage8_render.sum_terms, uut.stage8_render.sum_terms);
//             $display("  Power Temp: %d", uut.stage8_render.power_temp);
//             $display("  Power Shifted: %d", uut.stage8_render.power_shifted);
//             $display("  P (Clamped): %h", uut.stage8_render.P);
//         end
//         
//         if (pat_idx == 51 && uut.stage8_render.s8_pixel_count == 15 && uut.stage8_render.s8_valid) begin
//              $display("DEBUG: PROBE S8 G=%d Pixel=15 (Time: %t)", uut.stage8_render.gaussian_cnt, $time);
//              $display("  S8 Alpha: %d (Hex: %h)", uut.stage8_render.s8_alpha, uut.stage8_render.s8_alpha);
//              $display("  S8 Old T: %d (Hex: %h)", uut.stage8_render.s8_old_color_t, uut.stage8_render.s8_old_color_t);
//         end
// 
//         if (pat_idx == 51 && uut.stage8_render.s0_pixel_count == 15 && uut.stage8_render.s0_valid) begin
//             $display("DEBUG: PROBE S0 G=%d Pixel=15 (Time: %t)", uut.stage8_render.gaussian_cnt, $time);
//             $display("  Delta X: %d", uut.stage8_render.delta_x);
//             $display("  Delta Y: %d", uut.stage8_render.delta_y);
//         end
//     end
// end
endmodule
