module Stage7_Sort #(
    parameter BIT_WIDTH = 16
)(
    input wire clk, 
    input wire srst_n,
    input wire enable,
    input wire [6:0] gaussian_num, // 例如 64
    output wire valid, 
    output wire [2*BIT_WIDTH-1:0] sort_list, // 輸出目前比較的Debug用或寫回用兩個值()
    input  [31:0] sram_rdata_sort,
    output reg [31:0] sram_wdata_sort,
    output reg [5:0] sram_raddr_sort,
    output reg [5:0] sram_waddr_sort,
    output reg sram_wen_sort
);

    //========================
    // DECLARE
    //========================

    localparam IDLE  = 3'b000;
    localparam READ  = 3'b001;
    localparam SORT  = 3'b010;
    localparam WRITE = 3'b011;
    localparam DONE  = 3'b100;

    reg [2:0] state, state_n;
    reg [5:0] read_counter, read_counter_n;
    
    // Bitonic Sort Loop Counters
    reg [5:0] stage_counter, stage_counter_n; // 0 to log2(N)-1
    reg [5:0] round_counter, round_counter_n; // stage down to 0
    reg [5:0] sort_counter, sort_counter_n;   // 0 to N/2 - 1
    
    reg [2:0] max_stage_num; 

    // 計算總共有幾個 Stage (log2(N))
    always @(*) begin 
        if     (gaussian_num==2)  max_stage_num = 3'd0; // Stage 0 only
        else if(gaussian_num==4)  max_stage_num = 3'd1; // Stage 0, 1
        else if(gaussian_num==8)  max_stage_num = 3'd2;
        else if(gaussian_num==16) max_stage_num = 3'd3;
        else if(gaussian_num==32) max_stage_num = 3'd4;
        else if(gaussian_num==64) max_stage_num = 3'd5;
        else                      max_stage_num = 3'd0;
    end

    //========================
    // BITONIC ADDRESS CALCULATION logic
    //========================
    wire [5:0] pair_dist;
    assign pair_dist = 1'b1 << round_counter; // 距離: 1, 2, 4, ...

    wire [5:0] idx_a;
    wire [5:0] idx_b;
    
    assign idx_a = ( (sort_counter & ~(pair_dist - 1)) << 1 ) | (sort_counter & (pair_dist - 1));
    assign idx_b = idx_a | pair_dist;

    //========================
    // DATAPATH & SORT LOGIC
    //========================

    reg [2*BIT_WIDTH-1:0] data_A, data_A_n;
    reg [2*BIT_WIDTH-1:0] data_B, data_B_n;
    wire [BIT_WIDTH-1:0] val_A, val_B, index_A, index_B;
    assign val_A = data_A[BIT_WIDTH-1:0];
    assign index_A = data_A[2*BIT_WIDTH-1:BIT_WIDTH];
    assign val_B = data_B[BIT_WIDTH-1:0];
    assign index_B = data_B[2*BIT_WIDTH-1:BIT_WIDTH];
    
    // Bitonic Merge 需要根據當前的大區塊(Stage)決定是升冪還是降冪
    // 檢查 idx_a 的第 (stage_counter + 1) 個 bit
    wire dir_descending; // 1: Descending, 0: Ascending
    assign dir_descending = (idx_a >> (stage_counter + 1)) & 1'b1;

    reg swap;
    always @(*) begin
        if (dir_descending) begin
            // 降冪需求: 如果 A < B 則交換 (讓大的在前)
            if(val_A < val_B) swap = 1'b1; 
            else if( (val_A == val_B) && (index_A < index_B) ) swap = 1'b1;
            else swap = 1'b0;
        end else begin
            // 升冪需求: 如果 A > B 則交換
            if(val_A > val_B) swap = 1'b1;
            else if( (val_A == val_B) && (index_A > index_B) ) swap = 1'b1;
            else swap = 1'b0;
        end
    end

    reg [2*BIT_WIDTH-1:0] out_A;
    reg [2*BIT_WIDTH-1:0] out_B;

    always @(*) begin
        if (swap) begin
            out_A = {data_B[2*BIT_WIDTH-1:BIT_WIDTH] , val_B};
            out_B = {data_A[2*BIT_WIDTH-1:BIT_WIDTH] , val_A};
        end else begin
            out_A = {data_A[2*BIT_WIDTH-1:BIT_WIDTH] , val_A};
            out_B = {data_B[2*BIT_WIDTH-1:BIT_WIDTH] , val_B};
        end
    end
    
    assign sort_list = {out_A, out_B}; // 用於寫回

    //========================
    // FSM
    //========================

    always @(*) begin
        state_n = state;
        sram_wen_sort = 1'b1; // Default Read (Active Low usually? 假設 1 是 READ, 0 是 WRITE)
        
        sram_waddr_sort = 0;
        sram_wdata_sort = 0;
        sram_raddr_sort = 0;

        read_counter_n  = read_counter;
        stage_counter_n = stage_counter;
        round_counter_n = round_counter;
        sort_counter_n  = sort_counter;
        data_A_n = data_A;
        data_B_n = data_B;

        case(state)
            IDLE: begin
                if(enable && gaussian_num==1)begin
                    state_n = DONE; 
                    stage_counter_n = 0;
                    round_counter_n = 0; 
                    sort_counter_n = 0;
                end else if (enable) begin
                    state_n = READ;
                    stage_counter_n = 0;
                    round_counter_n = 0; // First stage, round starts at 0
                    sort_counter_n = 0;
                end
            end

            READ: begin
                // SRAM Read Sequence
                read_counter_n = read_counter + 1;
                case(read_counter)
                    0: begin
                        sram_raddr_sort = idx_a; // Cycle 0: Send Addr A
                    end
                    1: begin
                        sram_raddr_sort = idx_b; // Cycle 1: Send Addr B
                        // Data A 通常在 Cycle 1 結束或 Cycle 2 開始時可用 (取決於 SRAM 類型)
                    end
                    3: begin
                        data_A_n = sram_rdata_sort; // Capture Data A (from addr sent at 0)
                        sram_raddr_sort = 0; // Stop reading
                    end
                    4: begin
                        data_B_n = sram_rdata_sort; // Capture Data B (from addr sent at 1)
                        state_n = SORT;
                        read_counter_n = 0;
                    end
                endcase
            end

            SORT: begin
                state_n = WRITE;
            end

            WRITE: begin 
                read_counter_n = read_counter + 1;
                sram_wen_sort = 0; // Enable Write
                
                case(read_counter)
                    0: begin 
                        sram_waddr_sort = idx_a;
                        sram_wdata_sort = out_A; // 假設 SRAM 32bit 但存 16bit 資料
                    end
                    1: begin 
                        sram_waddr_sort = idx_b;
                        sram_wdata_sort = out_B;
                        
                        read_counter_n = 0; 
                        
                        // === Loop Control Logic ===
                        // 1. 完成一對 (Pair)
                        if(sort_counter == (gaussian_num/2 - 1)) begin
                            sort_counter_n = 0;
                            
                            // 2. 完成一個 Round (Step)
                            if (round_counter == 0) begin
                                
                                // 3. 完成一個 Stage
                                if (stage_counter == {3'b0, max_stage_num}) begin
                                    state_n = DONE;
                                end else begin
                                    // Next Stage
                                    state_n = READ;
                                    stage_counter_n = stage_counter + 1;
                                    round_counter_n = stage_counter + 1; // Round 重置為新的 Stage 值
                                end
                            end else begin
                                state_n = READ;
                                round_counter_n = round_counter - 1;
                            end
                        end else begin
                            // Next Pair
                            state_n = READ;
                            sort_counter_n = sort_counter + 1;
                        end 
                    end
                endcase
            end
            
            DONE: begin
                sram_wen_sort = 1; // Back to Read mode
                state_n = IDLE;
                sort_counter_n = 0;  
                round_counter_n = 0;
                stage_counter_n = 0;
            end
        endcase
    end

    // Sequential Logic
    always @(posedge clk) begin
        if (!srst_n) begin
            state <= IDLE;
            read_counter <= 0;
            stage_counter <= 0;
            round_counter <= 0;
            sort_counter <= 0;
            data_A <= 0;
            data_B <= 0;
        end else begin
            state <= state_n;
            read_counter <= read_counter_n;
            stage_counter <= stage_counter_n;
            round_counter <= round_counter_n;
            sort_counter <= sort_counter_n;
            data_A <= data_A_n;
            data_B <= data_B_n;
        end
    end

    assign valid = (state == DONE);

endmodule