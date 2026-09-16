module Stage3_Campos #(
    parameter BIT_WIDTH = 16, 
    parameter FLOAT_WIDTH = 6,
    parameter PRODUCT_WIDTH = 32
) (
    input clk,
    input srst_n,
    input enable, 
    output valid,
    input [6:0] gaussian_cnt, 
    input [6:0] gaussian_num,
    input [9*PRODUCT_WIDTH-1:0] campos_product,
    output reg [6:0] valid_gaussian_num,

    // SRAM Interface (Input Data) - 64-bit
    input  [4*BIT_WIDTH-1:0] sram_rdata_input,
    output [4*BIT_WIDTH-1:0] sram_wdata_input,
    output [15:0]  sram_raddr_input,
    output [15:0]  sram_waddr_input,

    // SRAM Interface (Sort Data)  - 48 bit 
    //input [2*BIT_WIDTH-1:0] sram_rdata_sort,
    output reg [2*BIT_WIDTH-1:0] sram_wdata_sort,
    //output reg [5:0] sram_raddr_sort,
    output reg [5:0] sram_waddr_sort,
    output reg sram_wen_sort,

    // SRAM Interface (View Matrix) - 64-bit
    input  [4*BIT_WIDTH-1:0] sram_rdata_view,
    output  [4*BIT_WIDTH-1:0] sram_wdata_view,
    output [15:0] sram_raddr_view,
    output [15:0] sram_waddr_view,
    output reg [3*BIT_WIDTH-1:0] campos,

    output reg signed [9*BIT_WIDTH-1:0] campos_multiplicand,
    output reg signed [9*BIT_WIDTH-1:0] campos_multiplier

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
    reg [3*BIT_WIDTH-1:0] campos_n;
    reg [6:0] valid_gaussian_num_n;

    // declare wire for position
    wire signed [BIT_WIDTH-1:0] posx; 
    wire signed [BIT_WIDTH-1:0] posy;
    wire signed [BIT_WIDTH-1:0] posz;
    // declare wire for view matrixs
    wire signed [BIT_WIDTH-1:0] view_0, view_1, view_2, view_3;
    reg signed [BIT_WIDTH-1:0] campos_x, campos_y, campos_z;
    reg signed [BIT_WIDTH-1:0] campos_x_n, campos_y_n, campos_z_n;
    reg signed [33:0] acc_x, acc_y, acc_z;
    wire signed [2*BIT_WIDTH-1:0] campos_temp;
    wire signed [BIT_WIDTH-1:0] campos_q;
    
    //========================
    // DATAPATH
    //========================
    assign view_0 = sram_rdata_view[BIT_WIDTH-1:0];
    assign view_1 = sram_rdata_view[2*BIT_WIDTH-1:BIT_WIDTH];
    assign view_2 = sram_rdata_view[3*BIT_WIDTH-1:2*BIT_WIDTH];
    assign view_3 = sram_rdata_view[4*BIT_WIDTH-1:3*BIT_WIDTH];
    assign posx = sram_rdata_input[BIT_WIDTH-1:0];
    assign posy = sram_rdata_input[2*BIT_WIDTH-1:BIT_WIDTH];
    assign posz = sram_rdata_input[3*BIT_WIDTH-1:2*BIT_WIDTH];

    assign sram_raddr_input = 0 + 4*gaussian_cnt; // hardcode accessing address0 (position of first gaussian )
    assign sram_wdata_input = 0;
    assign sram_waddr_input = 0;
    
    assign sram_waddr_view = 0;
    // assign sram_raddr_view = 0;
    assign sram_wdata_view = 0;


    always@(posedge clk)begin 
        if(!srst_n)begin 
            state <= IDLE;
            read_count <= 4'b0000;
            compute_count <= 4'd0;
            campos_x <= 0;
            campos_y <= 0;
            campos_z <= 0;
            campos <= 0;
            valid_gaussian_num <= gaussian_num;
        end else begin 
            state <= state_n;
            read_count <= read_count_n;
            compute_count <= compute_count_n;
            campos_x <= campos_x_n;
            campos_y <= campos_y_n;
            campos_z <= campos_z_n;
            campos <= campos_n;
            valid_gaussian_num <= valid_gaussian_num_n;
        end
    end

    always@* begin
        campos_multiplicand = {view_0, view_1, view_2, view_3, 80'b0};
        campos_multiplier = {posx, posy, posz, 16'b1, 80'b0};
    end

    always@(*)begin 
        state_n = state;
        read_count_n = read_count;
        compute_count_n = compute_count;
        campos_n = campos;
        valid_gaussian_num_n = valid_gaussian_num;

        acc_x = 0; acc_y = 0; acc_z = 0;
        campos_x_n = campos_x; campos_y_n = campos_y; campos_z_n = campos_z;

        // sram sort write default value
        sram_wen_sort = 1;
        sram_waddr_sort = 0;
        sram_wdata_sort = 0; 
        case(state)
            IDLE: begin
                read_count_n = 0; 
                if(enable) begin
                    state_n = READ;
                    valid_gaussian_num_n = gaussian_num;
                end
            end
            READ: begin
                state_n = READ;
                read_count_n = read_count + 1;
                if(read_count==2) state_n = COMPUTE; // 三個cycle的latency(檔input, output flipflop) , 加上送資料的兩個cycle
            end
            COMPUTE: begin
                case (compute_count)
                    0: begin
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    1: begin
                        acc_x = campos_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH] +
                                campos_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH] + 
                                campos_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH] +
                                (campos_product[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH] <<< 6);
                        campos_x_n = (acc_x + (1<<5)) >>> 6;

                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    2: begin
                        acc_y = campos_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH] +
                                campos_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH] + 
                                campos_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH] +
                                (campos_product[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH] <<< 6);
                        campos_y_n = (acc_y + (1<<5)) >>> 6;

                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    3: begin
                        acc_z = campos_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH] +
                                campos_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH] + 
                                campos_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH] +
                                (campos_product[6*PRODUCT_WIDTH-1:5*PRODUCT_WIDTH] <<< 6);
                        campos_z_n = (acc_z + (1<<5)) >>> 6;

                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    4: begin
                        compute_count_n = 0;
                        campos_n = {campos_x, campos_y, campos_z};
                        // write data to sort sram 
                        if(campos_z < 13) begin // culled if smaller than 0.2 (0.2 * 64 = 12.8 -> 13) 
                            valid_gaussian_num_n = valid_gaussian_num - 1; // need to render less one gaussian 
                            sram_wdata_sort = { 9'b0, gaussian_cnt, 16'hffff}; // use extreme large value to cull 
                        end else begin 
                            sram_wdata_sort = { 25'b0, gaussian_cnt, campos_z};
                        end
                        sram_waddr_sort = gaussian_cnt;
                        sram_wen_sort = 0;
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

    // set address logic 
    // 前四個cycle分別送出addr 0, 1, 2
    assign sram_raddr_view = read_count;
    // read view matrix, three cycle latency

    /*
    always@(*)begin 
        acc_x = 0; acc_y = 0; acc_z = 0;
        campos_x_n = campos_x; campos_y_n = campos_y; campos_z_n = campos_z;
        
        
        case(compute_count)
            0: begin
                acc_x = posx * view_0 + posy * view_1 + posz * view_2 + (view_3 <<< 6);
                campos_x_n = (acc_x + (1<<5)) >>> 6;
            end
            1: begin
                acc_y = posx * view_0 + posy * view_1 + posz * view_2 + (view_3 <<< 6);
                campos_y_n = (acc_y + (1<<5)) >>> 6;
            end
            2: begin
                acc_z = posx * view_0 + posy * view_1 + posz * view_2 + (view_3 <<< 6);
                campos_z_n = (acc_z + (1<<5)) >>> 6;
            end
            default:;
        endcase
        
    end
    */
    
    // output 
    assign valid = (state==DONE);

endmodule
