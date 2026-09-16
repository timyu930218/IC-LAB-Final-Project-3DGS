module Stage2_Conv #(
    parameter BIT_WIDTH = 16, 
    parameter FLOAT_WIDTH = 6,
    parameter PRODUCT_WIDTH = 32
) (
    input clk,
    input srst_n,
    input enable, 
    input [6:0] gaussian_cnt,
    input [9*PRODUCT_WIDTH-1:0] conv_product,

    output valid,

    // SRAM Interface (Input Data) - 64-bit
    input  [4*BIT_WIDTH-1:0] sram_rdata_input,
    output [4*BIT_WIDTH-1:0] sram_wdata_input,
    output [15:0]  sram_raddr_input,
    output [15:0]  sram_waddr_input,

    input [9*BIT_WIDTH-1:0] rotation_matrix, 
    output reg [9*BIT_WIDTH-1:0] conv_matrix,

    output reg signed [9*BIT_WIDTH-1:0] conv_multiplicand,
    output reg signed [9*BIT_WIDTH-1:0] conv_multiplier

);

    //========================
    // DECLARE
    //========================

    // declare parameter for fsm 
    localparam  IDLE = 3'b000, 
                READ = 3'b001, 
                COMPUTE = 3'b010, 
                DONE = 3'b011;
    reg [2:0] state, state_n;
    reg [3:0] read_count, read_count_n;
    reg [3:0] compute_count, compute_count_n;

    // declare wire for scaling factor
    wire signed [BIT_WIDTH-1:0] sz; 
    wire signed [BIT_WIDTH-1:0] sy;
    wire signed [BIT_WIDTH-1:0] sx;
    // declare wire for matrixs
    wire signed [BIT_WIDTH-1:0] r_00, r_01, r_02, r_10, r_11, r_12, r_20, r_21, r_22;
    
    reg signed [2*BIT_WIDTH-1:0] m_00, m_01, m_02, m_10, m_11, m_12, m_20, m_21, m_22;
    reg signed [2*BIT_WIDTH-1:0] m_00_n, m_01_n, m_02_n, m_10_n, m_11_n, m_12_n, m_20_n, m_21_n, m_22_n;

    reg signed [BIT_WIDTH-1:0] m_00_q, m_01_q, m_02_q, m_10_q, m_11_q, m_12_q, m_20_q, m_21_q, m_22_q;
    reg signed [BIT_WIDTH-1:0] m_00_q_n, m_01_q_n, m_02_q_n, m_10_q_n, m_11_q_n, m_12_q_n, m_20_q_n, m_21_q_n, m_22_q_n;

    reg signed [2*BIT_WIDTH+1:0] conv_00_n, conv_01_n, conv_02_n, conv_11_n, conv_12_n, conv_22_n;  //I change the bit width
    reg signed [2*BIT_WIDTH-1:0] conv_00, conv_01, conv_02, conv_11, conv_12, conv_22;


    reg signed [BIT_WIDTH-1:0] conv_00_q, conv_01_q, conv_02_q, conv_11_q, conv_12_q, conv_22_q;


    reg [9*BIT_WIDTH-1:0] conv_matrix_n;

    
    //========================
    // DATAPATH
    //========================
    
    // read input from sram 
    assign sx = sram_rdata_input[BIT_WIDTH-1:0];
    assign sy = sram_rdata_input[2*BIT_WIDTH-1:BIT_WIDTH];
    assign sz = sram_rdata_input[3*BIT_WIDTH-1:2*BIT_WIDTH];

    assign r_00 = rotation_matrix[9*BIT_WIDTH-1:8*BIT_WIDTH];
    assign r_01 = rotation_matrix[8*BIT_WIDTH-1:7*BIT_WIDTH];
    assign r_02 = rotation_matrix[7*BIT_WIDTH-1:6*BIT_WIDTH];
    assign r_10 = rotation_matrix[6*BIT_WIDTH-1:5*BIT_WIDTH];
    assign r_11 = rotation_matrix[5*BIT_WIDTH-1:4*BIT_WIDTH];
    assign r_12 = rotation_matrix[4*BIT_WIDTH-1:3*BIT_WIDTH];
    assign r_20 = rotation_matrix[3*BIT_WIDTH-1:2*BIT_WIDTH];
    assign r_21 = rotation_matrix[2*BIT_WIDTH-1:1*BIT_WIDTH];
    assign r_22 = rotation_matrix[1*BIT_WIDTH-1:0];

    // calculate M = R * S
    /*
    assign m_00 = sx * r_00;
    assign m_01 = sy * r_01;
    assign m_02 = sz * r_02;
    assign m_10 = sx * r_10;
    assign m_11 = sy * r_11;
    assign m_12 = sz * r_12;
    assign m_20 = sx * r_20;
    assign m_21 = sy * r_21;
    assign m_22 = sz * r_22;
    */

    // quantize M 
    /*
    assign m_00_q = (m_00 + (1<<5)) >>> 6;
    assign m_01_q = (m_01 + (1<<5)) >>> 6;
    assign m_02_q = (m_02 + (1<<5)) >>> 6;
    assign m_10_q = (m_10 + (1<<5)) >>> 6;
    assign m_11_q = (m_11 + (1<<5)) >>> 6;
    assign m_12_q = (m_12 + (1<<5)) >>> 6;
    assign m_20_q = (m_20 + (1<<5)) >>> 6;
    assign m_21_q = (m_21 + (1<<5)) >>> 6;
    assign m_22_q = (m_22 + (1<<5)) >>> 6;
    */

    // calculate M * M.T
    // 因為M*M.T會是對稱矩陣，所以只用計算上三角，我先幫你註解掉了
    //assign conv_00_n = m_00_q*m_00_q + m_01_q*m_01_q + m_02_q*m_02_q;
    //assign conv_01_n = m_00_q*m_10_q + m_01_q*m_11_q + m_02_q*m_12_q;
    //assign conv_02_n = m_00_q*m_20_q + m_01_q*m_21_q + m_02_q*m_22_q;

    // assign conv_10_n = m_10_q*m_00_q + m_11_q*m_01_q + m_12_q*m_02_q;

    //assign conv_11_n = m_10_q*m_10_q + m_11_q*m_11_q + m_12_q*m_12_q;
    //assign conv_12_n = m_10_q*m_20_q + m_11_q*m_21_q + m_12_q*m_22_q;

    // assign conv_20_n = m_20_q*m_00_q + m_21_q*m_01_q + m_22_q*m_02_q;
    // assign conv_21_n = m_20_q*m_10_q + m_21_q*m_11_q + m_22_q*m_12_q;

    //assign conv_22_n = m_20_q*m_20_q + m_21_q*m_21_q + m_22_q*m_22_q;

    // quantize M * M.T

    /*
    assign conv_00_q = (conv_00_n + (1<<5)) >>> 6;
    assign conv_01_q = (conv_01_n + (1<<5)) >>> 6;
    assign conv_02_q = (conv_02_n + (1<<5)) >>> 6;
    assign conv_10_q = (conv_10_n + (1<<5)) >>> 6;
    assign conv_11_q = (conv_11_n + (1<<5)) >>> 6;
    assign conv_12_q = (conv_12_n + (1<<5)) >>> 6;
    assign conv_20_q = (conv_20_n + (1<<5)) >>> 6;
    assign conv_21_q = (conv_21_n + (1<<5)) >>> 6;
    assign conv_22_q = (conv_22_n + (1<<5)) >>> 6;
    */
    // assign conv_matrix = {conv_00_q, conv_01_q, conv_02_q, conv_10_q, conv_11_q, conv_12_q, conv_20_q, conv_21_q, conv_22_q};


    //========================
    // FSM (control unit )
    //========================
    
    // Sram Control logic 
    // 如果後面要讀取其他的gaussian，要再拉fsm的訊號近來選要讀哪一個gaussian的參數
    assign sram_raddr_input = 1+ gaussian_cnt*4; // hardcode accessing address1 (scaling factor of first gaussian )
    assign sram_wdata_input = 0;
    assign sram_waddr_input = 0;

    always@(posedge clk)begin 
        if(!srst_n)begin 
            state <= IDLE;
            read_count <= 4'b0000;
            compute_count <= 4'd0;
            conv_matrix <= 0;
        end else begin 
            state <= state_n;
            read_count <= read_count_n;
            compute_count <= compute_count_n;
            conv_matrix <= conv_matrix_n;
            m_00 <= m_00_n;
            m_01 <= m_01_n;
            m_02 <= m_02_n;
            m_10 <= m_10_n;
            m_11 <= m_11_n;
            m_12 <= m_12_n;
            m_20 <= m_20_n;
            m_21 <= m_21_n;
            m_22 <= m_22_n;

            m_00_q <= m_00_q_n;
            m_01_q <= m_01_q_n;
            m_02_q <= m_02_q_n;
            m_10_q <= m_10_q_n;
            m_11_q <= m_11_q_n;
            m_12_q <= m_12_q_n;
            m_20_q <= m_20_q_n;
            m_21_q <= m_21_q_n;
            m_22_q <= m_22_q_n;

            conv_00 <= conv_00_n;
            conv_01 <= conv_01_n;
            conv_02 <= conv_02_n;
            conv_11 <= conv_11_n;
            conv_12 <= conv_12_n;
            conv_22 <= conv_22_n;
        end
    end

    always@* begin
        conv_multiplicand = 0;
        conv_multiplier = 0;
        case (compute_count)
            0: begin
                conv_multiplicand = {sx, sy, sz, sx, sy, sz, sx, sy, sz};
                conv_multiplier = {r_00, r_01, r_02, r_10, r_11, r_12, r_20, r_21, r_22};
            end

            3: begin
                conv_multiplicand = {m_00_q, m_01_q, m_02_q, m_00_q, m_01_q, m_02_q, m_00_q, m_01_q, m_02_q};
                conv_multiplier = {m_00_q, m_01_q, m_02_q, m_10_q, m_11_q, m_12_q, m_20_q, m_21_q, m_22_q};
            end

            4: begin
                conv_multiplicand = {m_10_q, m_11_q, m_12_q, m_10_q, m_11_q, m_12_q, m_20_q, m_21_q, m_22_q};
                conv_multiplier = {m_10_q, m_11_q, m_12_q, m_20_q, m_21_q, m_22_q, m_20_q, m_21_q, m_22_q};
            end
            default:;
        endcase
    end

    always@(*)begin 
        state_n = state;
        read_count_n = read_count;
        compute_count_n = compute_count;
        conv_matrix_n = conv_matrix;
            m_00_n = m_00;
            m_01_n = m_01;
            m_02_n = m_02;
            m_10_n = m_10;
            m_11_n = m_11;
            m_12_n = m_12;
            m_20_n = m_20;
            m_21_n = m_21;
            m_22_n = m_22;

            m_00_q_n = m_00_q;
            m_01_q_n = m_01_q;
            m_02_q_n = m_02_q;
            m_10_q_n = m_10_q;
            m_11_q_n = m_11_q;
            m_12_q_n = m_12_q;
            m_20_q_n = m_20_q;
            m_21_q_n = m_21_q;
            m_22_q_n = m_22_q;

            conv_00_n = conv_00;
            conv_01_n = conv_01;
            conv_02_n = conv_02;
            conv_11_n = conv_11;
            conv_12_n = conv_12;
            conv_22_n = conv_22;


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
                    0: begin //compute m

                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    1: begin
                        m_00_n = conv_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH];
                        m_01_n = conv_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH];
                        m_02_n = conv_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH];
                        m_10_n = conv_product[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH];
                        m_11_n = conv_product[5*PRODUCT_WIDTH-1:4*PRODUCT_WIDTH];
                        m_12_n = conv_product[4*PRODUCT_WIDTH-1:3*PRODUCT_WIDTH];
                        m_20_n = conv_product[3*PRODUCT_WIDTH-1:2*PRODUCT_WIDTH];
                        m_21_n = conv_product[2*PRODUCT_WIDTH-1:1*PRODUCT_WIDTH];
                        m_22_n = conv_product[1*PRODUCT_WIDTH-1:0*PRODUCT_WIDTH];

                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    2: begin
                        m_00_q_n = (m_00 + (1<<5)) >>> 6;
                        m_01_q_n = (m_01 + (1<<5)) >>> 6;
                        m_02_q_n = (m_02 + (1<<5)) >>> 6;
                        m_10_q_n = (m_10 + (1<<5)) >>> 6;
                        m_11_q_n = (m_11 + (1<<5)) >>> 6;
                        m_12_q_n = (m_12 + (1<<5)) >>> 6;
                        m_20_q_n = (m_20 + (1<<5)) >>> 6;
                        m_21_q_n = (m_21 + (1<<5)) >>> 6;
                        m_22_q_n = (m_22 + (1<<5)) >>> 6;

                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    3: begin //compute conv
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    4: begin
                        conv_00_n = conv_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH] + 
                                    conv_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH] + 
                                    conv_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH];
                        conv_01_n = conv_product[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH] + 
                                    conv_product[5*PRODUCT_WIDTH-1:4*PRODUCT_WIDTH] + 
                                    conv_product[4*PRODUCT_WIDTH-1:3*PRODUCT_WIDTH];
                        conv_02_n = conv_product[3*PRODUCT_WIDTH-1:2*PRODUCT_WIDTH] + 
                                    conv_product[2*PRODUCT_WIDTH-1:1*PRODUCT_WIDTH] + 
                                    conv_product[1*PRODUCT_WIDTH-1:0*PRODUCT_WIDTH];

                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;

                    end

                    5: begin
                        conv_11_n = conv_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH] + 
                                    conv_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH] + 
                                    conv_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH];
                        conv_12_n = conv_product[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH] + 
                                    conv_product[5*PRODUCT_WIDTH-1:4*PRODUCT_WIDTH] + 
                                    conv_product[4*PRODUCT_WIDTH-1:3*PRODUCT_WIDTH];
                        conv_22_n = conv_product[3*PRODUCT_WIDTH-1:2*PRODUCT_WIDTH] + 
                                    conv_product[2*PRODUCT_WIDTH-1:1*PRODUCT_WIDTH] + 
                                    conv_product[1*PRODUCT_WIDTH-1:0*PRODUCT_WIDTH];

                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;            
                    end
                    
                    6: begin
                        conv_00_q = (conv_00 + (1<<5)) >>> 6;
                        conv_01_q = (conv_01 + (1<<5)) >>> 6;
                        conv_02_q = (conv_02 + (1<<5)) >>> 6;
                        conv_11_q = (conv_11 + (1<<5)) >>> 6;
                        conv_12_q = (conv_12 + (1<<5)) >>> 6;
                        conv_22_q = (conv_22 + (1<<5)) >>> 6;

                        conv_matrix_n = {conv_00_q, conv_01_q, conv_02_q, 
                                        conv_01_q, conv_11_q, conv_12_q, 
                                        conv_02_q, conv_12_q, conv_22_q};
                        compute_count_n = 0;                
                        state_n = DONE;
                    end

                    default:;
                endcase 
                // 我現在寫的版本是一個cycle算出來，你在優化一下這邊
            end
            DONE: begin
                state_n = IDLE;
            end

            default:;
        endcase
    end
    
    // output 
    assign valid = (state==DONE);

endmodule
