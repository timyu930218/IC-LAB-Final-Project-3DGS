module multiply_long #(
    parameter BW_PER_MCD   = 40, 
    parameter BW_PER_MER   = 16,
    parameter N_MULT = 9
) (
    input wire clk,
    //input wire srst_n, 
    input wire signed [N_MULT*BW_PER_MCD-1:0] in_a,
    input wire signed [N_MULT*BW_PER_MER-1:0] in_b,
    output wire signed [N_MULT*(BW_PER_MCD+BW_PER_MER)-1:0] out
);

    localparam OUT_BW = BW_PER_MCD + BW_PER_MER; // 56 bits
    localparam CHUNK  = 8;
    localparam CH_A   = BW_PER_MCD / CHUNK; // 5
    localparam CH_B   = BW_PER_MER / CHUNK; // 2

    genvar g;
    generate
        for (g = 0; g < N_MULT; g = g + 1) begin : GEN_MULT
            
            wire signed [BW_PER_MCD-1:0] curr_a = in_a[(g+1)*BW_PER_MCD-1 : g*BW_PER_MCD];
            wire signed [BW_PER_MER-1:0] curr_b = in_b[(g+1)*BW_PER_MER-1 : g*BW_PER_MER];

            wire signed [CHUNK:0] a_seg [0:CH_A-1];
            wire signed [CHUNK:0] b_seg [0:CH_B-1];

            assign a_seg[0] = $signed({1'b0, curr_a[0*CHUNK +: CHUNK]});
            assign a_seg[1] = $signed({1'b0, curr_a[1*CHUNK +: CHUNK]});
            assign a_seg[2] = $signed({1'b0, curr_a[2*CHUNK +: CHUNK]});
            assign a_seg[3] = $signed({1'b0, curr_a[3*CHUNK +: CHUNK]});
            assign a_seg[4] = $signed(curr_a[4*CHUNK +: CHUNK]); 

            assign b_seg[0] = $signed({1'b0, curr_b[0*CHUNK +: CHUNK]});
            assign b_seg[1] = $signed(curr_b[1*CHUNK +: CHUNK]);

            // Stage 1: Multiplication

            reg signed [2*CHUNK:0] s1_pp [0:CH_A-1][0:CH_B-1];
            
            integer i, j;
            always @(posedge clk) begin
                    for (i = 0; i < CH_A; i = i + 1) begin
                        for (j = 0; j < CH_B; j = j + 1) s1_pp[i][j] <= a_seg[i] * b_seg[j];
                    end
                /*
                if (!srst_n) begin
                    for (i = 0; i < CH_A; i = i + 1) begin
                        for (j = 0; j < CH_B; j = j + 1) s1_pp[i][j] <= 'd0;
                    end
                end else begin
                    for (i = 0; i < CH_A; i = i + 1) begin
                        for (j = 0; j < CH_B; j = j + 1) s1_pp[i][j] <= a_seg[i] * b_seg[j];
                    end
                end
                */
            end

            // Stage 2: Partial Summation

            reg signed [2*CHUNK+1:0] s2_sum_weight [0:CH_A+CH_B-2]; 
            
            integer k;
            always @(posedge clk) begin
                /*
                if (!srst_n) begin
                     for (k = 0; k <= CH_A+CH_B-2; k = k + 1) s2_sum_weight[k] <= 'd0;
                end else begin
                    s2_sum_weight[0] <= s1_pp[0][0];
                    s2_sum_weight[1] <= s1_pp[1][0] + s1_pp[0][1];
                    s2_sum_weight[2] <= s1_pp[2][0] + s1_pp[1][1];
                    s2_sum_weight[3] <= s1_pp[3][0] + s1_pp[2][1];
                    s2_sum_weight[4] <= s1_pp[4][0] + s1_pp[3][1];
                    s2_sum_weight[5] <= s1_pp[4][1];
                end
                */


                    s2_sum_weight[0] <= s1_pp[0][0];
                    s2_sum_weight[1] <= s1_pp[1][0] + s1_pp[0][1];
                    s2_sum_weight[2] <= s1_pp[2][0] + s1_pp[1][1];
                    s2_sum_weight[3] <= s1_pp[3][0] + s1_pp[2][1];
                    s2_sum_weight[4] <= s1_pp[4][0] + s1_pp[3][1];
                    s2_sum_weight[5] <= s1_pp[4][1];

            end


            // Stage 3: Final Accumulation
            reg signed [OUT_BW-1:0] s3_final_out;
            
            wire signed [OUT_BW-1:0] sum_term_0;
            wire signed [OUT_BW-1:0] sum_term_1;
            wire signed [OUT_BW-1:0] sum_term_2;
            wire signed [OUT_BW-1:0] sum_term_3;
            wire signed [OUT_BW-1:0] sum_term_4;
            wire signed [OUT_BW-1:0] sum_term_5;
            
            // Weight 0: No shift. Sign Extend (56 - 18 = 38 bits)
            assign sum_term_0 = { {38{s2_sum_weight[0][17]}}, s2_sum_weight[0] };

            // Weight 1: Shift 8. Sign Extend (56 - 8 - 18 = 30 bits)
            assign sum_term_1 = { {30{s2_sum_weight[1][17]}}, s2_sum_weight[1], 8'd0 };

            // Weight 2: Shift 16. Sign Extend (56 - 16 - 18 = 22 bits)
            assign sum_term_2 = { {22{s2_sum_weight[2][17]}}, s2_sum_weight[2], 16'd0 };

            // Weight 3: Shift 24. Sign Extend (56 - 24 - 18 = 14 bits)
            assign sum_term_3 = { {14{s2_sum_weight[3][17]}}, s2_sum_weight[3], 24'd0 };

            // Weight 4: Shift 32. Sign Extend (56 - 32 - 18 = 6 bits)
            assign sum_term_4 = { { 6{s2_sum_weight[4][17]}}, s2_sum_weight[4], 32'd0 };

            // Weight 5: Shift 40. Overflow handling.
            // s2_sum_weight[5] 18 bits，左移 40 後總共需 58 bits
            // 但輸出只有 56 bits，所以最高 2 bits 會被切掉
            // 我們取後面 16 bits (15:0) 即可
            assign sum_term_5 = { s2_sum_weight[5][15:0], 40'd0 };

            always @(posedge clk) begin
                    s3_final_out <= sum_term_0 + sum_term_1 + sum_term_2 + 
                                    sum_term_3 + sum_term_4 + sum_term_5;
                /*
                if (!srst_n) begin
                    s3_final_out <= 'd0;
                end else begin
                    s3_final_out <= sum_term_0 + sum_term_1 + sum_term_2 + 
                                    sum_term_3 + sum_term_4 + sum_term_5;
                end
                */
            end

            assign out[(g+1)*OUT_BW-1 : g*OUT_BW] = s3_final_out;
        end
    endgenerate

endmodule