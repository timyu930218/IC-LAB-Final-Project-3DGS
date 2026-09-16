module Stage1_Rot #(
    parameter BIT_WIDTH = 16, 
    parameter FLOAT_WIDTH = 6,
    parameter PRODUCT_WIDTH = 32
) (
    input clk,
    input srst_n,
    input enable, 
    input [6:0] gaussian_cnt,
    input [9*PRODUCT_WIDTH-1:0] rot_product,

    output valid,

    // SRAM Interface (Input Data) - 64-bit
    input  [4*BIT_WIDTH-1:0] sram_rdata_input,
    output [4*BIT_WIDTH-1:0] sram_wdata_input,
    output [15:0]  sram_raddr_input,
    output [15:0]  sram_waddr_input,

    output reg [9*BIT_WIDTH-1:0] rotation_matrix,

    output reg signed [9*BIT_WIDTH-1:0] rot_multiplicand,
    output reg signed [9*BIT_WIDTH-1:0] rot_multiplier

);

    //========================
    // DECLARE
    //========================
    
    // control unit declare
    localparam IDLE = 3'b000,
               READ = 3'b001,
               COMPUTE = 3'b010,
               DONE = 3'b011; 
    reg [2:0] state  , state_n  ;
    reg [3:0] read_count, read_count_n; 
    reg [3:0] compute_count, compute_count_n; 

    // datapath declare 
    wire signed [BIT_WIDTH-1:0] w; 
    wire signed [BIT_WIDTH-1:0] x;
    wire signed [BIT_WIDTH-1:0] y;
    wire signed [BIT_WIDTH-1:0] z;
    wire signed [2*BIT_WIDTH-1:0] mat_00_n; 
    wire signed [2*BIT_WIDTH-1:0] mat_01_n;
    wire signed [2*BIT_WIDTH-1:0] mat_02_n;
    wire signed [2*BIT_WIDTH-1:0] mat_10_n;
    wire signed [2*BIT_WIDTH-1:0] mat_11_n;
    wire signed [2*BIT_WIDTH-1:0] mat_12_n;
    wire signed [2*BIT_WIDTH-1:0] mat_20_n;
    wire signed [2*BIT_WIDTH-1:0] mat_21_n;
    wire signed [2*BIT_WIDTH-1:0] mat_22_n;
    wire signed [BIT_WIDTH-1:0] mat_00; 
    wire signed [BIT_WIDTH-1:0] mat_01;
    wire signed [BIT_WIDTH-1:0] mat_02;
    wire signed [BIT_WIDTH-1:0] mat_10;
    wire signed [BIT_WIDTH-1:0] mat_11;
    wire signed [BIT_WIDTH-1:0] mat_12;
    wire signed [BIT_WIDTH-1:0] mat_20;
    wire signed [BIT_WIDTH-1:0] mat_21;
    wire signed [BIT_WIDTH-1:0] mat_22; 
    reg [9*BIT_WIDTH-1:0] rotation_matrix_n; 

    reg [9*PRODUCT_WIDTH-1:0] rot_product_reg;
    
    //========================
    // DATAPATH
    //========================

    // assign rotation_matrix = {mat_00, mat_01, mat_02, mat_10, mat_11, mat_12, mat_20, mat_21, mat_22};
    assign w = sram_rdata_input[BIT_WIDTH-1:0];
    assign x = sram_rdata_input[2*BIT_WIDTH-1:BIT_WIDTH];
    assign y = sram_rdata_input[3*BIT_WIDTH-1:2*BIT_WIDTH];
    assign z = sram_rdata_input[4*BIT_WIDTH-1:3*BIT_WIDTH];

    assign mat_00_n = (1<<12) - ((rot_product_reg[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH] + rot_product_reg[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH]) << 1) + (1<<5);
    assign mat_01_n = ((rot_product_reg[3*PRODUCT_WIDTH-1:2*PRODUCT_WIDTH] - rot_product_reg[4*PRODUCT_WIDTH-1:3*PRODUCT_WIDTH]) << 1) + (1<<5);
    assign mat_02_n = ((rot_product_reg[2*PRODUCT_WIDTH-1:1*PRODUCT_WIDTH] + rot_product_reg[5*PRODUCT_WIDTH-1:4*PRODUCT_WIDTH]) << 1) + (1<<5);
    assign mat_10_n = ((rot_product_reg[3*PRODUCT_WIDTH-1:2*PRODUCT_WIDTH] + rot_product_reg[4*PRODUCT_WIDTH-1:3*PRODUCT_WIDTH]) << 1) + (1<<5);
    assign mat_11_n = (1<<12) - ((rot_product_reg[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH] + rot_product_reg[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH]) << 1) + (1<<5);
    assign mat_12_n = ((rot_product_reg[1*PRODUCT_WIDTH-1:0*PRODUCT_WIDTH] - rot_product_reg[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH]) << 1) + (1<<5);
    assign mat_20_n = ((rot_product_reg[2*PRODUCT_WIDTH-1:1*PRODUCT_WIDTH] - rot_product_reg[5*PRODUCT_WIDTH-1:4*PRODUCT_WIDTH]) << 1) + (1<<5);
    assign mat_21_n = ((rot_product_reg[1*PRODUCT_WIDTH-1:0*PRODUCT_WIDTH] + rot_product_reg[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH]) << 1) + (1<<5);
    assign mat_22_n = (1<<12) - ((rot_product_reg[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH] + rot_product_reg[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH]) << 1) + (1<<5); 
    /*
    assign mat_00_n = (1<<12) - 2*(y*y + z*z) + (1<<5);
    assign mat_01_n = 2*(x*y - z*w) + (1<<5);
    assign mat_02_n = 2*(x*z + y*w) + (1<<5);
    assign mat_10_n = 2*(x*y + z*w) + (1<<5);
    assign mat_11_n = (1<<12) - 2*(x*x + z*z) + (1<<5);
    assign mat_12_n = 2*(y*z - x*w) + (1<<5);
    assign mat_20_n = 2*(x*z - y*w) + (1<<5);
    assign mat_21_n = 2*(y*z + x*w) + (1<<5);
    assign mat_22_n = (1<<12) - 2*(x*x + y*y) + (1<<5);
    */
    assign mat_00 = mat_00_n[21:6];
    assign mat_01 = mat_01_n[21:6];
    assign mat_02 = mat_02_n[21:6];
    assign mat_10 = mat_10_n[21:6];
    assign mat_11 = mat_11_n[21:6];
    assign mat_12 = mat_12_n[21:6];
    assign mat_20 = mat_20_n[21:6];
    assign mat_21 = mat_21_n[21:6];
    assign mat_22 = mat_22_n[21:6];

    //========================
    // FSM (control unit )
    //========================

    // assign default sram address for first gaussian
    // 如果之後要讀其他gaussian，要再拉fsm的訊號近來選要讀哪一個gaussian的參數
    assign sram_raddr_input = 2 + 4*gaussian_cnt; // iterate gaussian using gaussian cnt
    assign sram_wdata_input = 0;
    assign sram_waddr_input = 0;

    // Sequential Logic
    always @(posedge clk) begin
        if (!srst_n) begin
            state <= IDLE;
            read_count <= 4'b0000;
            compute_count <= 4'd0;
            rotation_matrix <= 0;
        end else begin
            state <= state_n;
            read_count <= read_count_n;
            rotation_matrix <= rotation_matrix_n;
            compute_count <= compute_count_n;
            rot_product_reg <= rot_product;
        end
    end


    always @* begin
        rot_multiplicand = 0;
        rot_multiplier = 0;
        case (compute_count)
            0: begin
                rot_multiplicand = {x, y, z, w, w, w, x, x, y};
                rot_multiplier = {x, y, z, x, y, z, y, z, z};
            end
            default:;
        endcase
    end

    // State Transition
    always @(*) begin
        state_n = state;
        read_count_n = read_count;
        compute_count_n = compute_count;
        rotation_matrix_n = rotation_matrix;
        //rot_product_reg = rot_product_reg;
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
                if(read_count==2) state_n = COMPUTE; // 三個cycle的latency(檔input, output flipflop) 
            end
            COMPUTE: begin
                case (compute_count)
                    0: begin    
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    1: begin
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    2: begin
                        compute_count_n = 0;
                        rotation_matrix_n = {mat_00, mat_01, mat_02, mat_10, mat_11, mat_12, mat_20, mat_21, mat_22};
                        state_n = DONE;
                    end

                    default:;
                endcase
                
            end
            DONE: begin
                state_n = IDLE;
            end
        endcase
    end

    // output logic
    assign valid = (state==DONE);

endmodule
