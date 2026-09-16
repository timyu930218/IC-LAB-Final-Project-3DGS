
module multiply #(
    parameter BW_PER_MCD = 16,
    parameter BW_PER_MER = 16,
    parameter N_MULT = 9
) (
    input wire clk, 
    input wire signed [N_MULT*BW_PER_MCD-1:0] in_a,
    input wire signed [N_MULT*BW_PER_MER-1:0] in_b,
    output wire signed [N_MULT*(BW_PER_MCD+BW_PER_MER)-1:0] out
);

    localparam OUT_BW = BW_PER_MCD + BW_PER_MER;
    
    localparam HALF_W = 8; 

    genvar i;
    generate
        for (i = 0; i < N_MULT; i = i + 1) begin : GEN_MULT

            // Slice inputs
            wire signed [BW_PER_MCD-1:0] current_a = in_a[(i+1)*BW_PER_MCD-1 : i*BW_PER_MCD];
            wire signed [BW_PER_MER-1:0] current_b = in_b[(i+1)*BW_PER_MER-1 : i*BW_PER_MER];

            // 拆分為 High (Signed) 與 Low (Unsigned)
            wire signed [HALF_W-1:0] a_h = current_a[BW_PER_MCD-1 : HALF_W];
            wire signed [HALF_W-1:0] b_h = current_b[BW_PER_MER-1 : HALF_W];

            // Low part: 
            wire [HALF_W-1:0] a_l = current_a[HALF_W-1 : 0];
            wire [HALF_W-1:0] b_l = current_b[HALF_W-1 : 0];

            // multiplication
            
            // P_LL: Unsigned * Unsigned -> Unsigned
            wire [2*HALF_W-1:0] p_ll = a_l * b_l;

            // P_LH: Unsigned * Signed -> Signed 
            wire signed [2*HALF_W-1:0] p_lh = $signed({1'b0, a_l}) * b_h;

            // P_HL: Signed * Unsigned -> Signed
            wire signed [2*HALF_W-1:0] p_hl = a_h * $signed({1'b0, b_l});

            // P_HH: Signed * Signed -> Signed
            wire signed [2*HALF_W-1:0] p_hh = a_h * b_h;

            // Pre-add middle terms
            wire signed [2*HALF_W:0] sum_mid_wire = p_lh + p_hl;

            reg signed [2*HALF_W-1:0] reg_p_hh;
            reg signed [2*HALF_W:0]   reg_sum_mid;
            reg        [2*HALF_W-1:0] reg_p_ll;

            always @(posedge clk) begin
                reg_p_hh    <= p_hh;
                reg_sum_mid <= sum_mid_wire;
                reg_p_ll    <= p_ll;
            end

            //Shift and final add
            
            wire signed [OUT_BW-1:0] final_product;
            
            assign final_product = (reg_p_hh <<< (2*HALF_W)) + 
                                   (reg_sum_mid <<< HALF_W) + 
                                   $signed({1'b0, reg_p_ll}); 

            assign out[(i+1)*OUT_BW-1 : i*OUT_BW] = final_product;
        end
    endgenerate

endmodule
