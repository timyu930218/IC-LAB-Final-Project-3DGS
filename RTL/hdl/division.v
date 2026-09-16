module division #(
    parameter A_WIDTH = 36,  // 36 bits
    parameter B_WIDTH = 36   // 36 bits
)(
    input  wire clk,
    input  wire srst_n,
    input  wire [A_WIDTH-1:0] a,      
    input  wire [B_WIDTH-1:0] b,      
    output wire [A_WIDTH-1:0] quotient, 
    output wire divide_by_0
);


    // 3 * 12 = 36 bits

    localparam STAGE_BITS = 3; 

    // Pipeline Registers 
    // Stage 1 
    reg [A_WIDTH+B_WIDTH-1:0] pipe1_work; reg [B_WIDTH-1:0] pipe1_b; reg pipe1_div0;
    // Stage 2 
    reg [A_WIDTH+B_WIDTH-1:0] pipe2_work; reg [B_WIDTH-1:0] pipe2_b; reg pipe2_div0;
    // Stage 3 
    reg [A_WIDTH+B_WIDTH-1:0] pipe3_work; reg [B_WIDTH-1:0] pipe3_b; reg pipe3_div0;
    // Stage 4 
    reg [A_WIDTH+B_WIDTH-1:0] pipe4_work; reg [B_WIDTH-1:0] pipe4_b; reg pipe4_div0;
    // Stage 5 
    reg [A_WIDTH+B_WIDTH-1:0] pipe5_work; reg [B_WIDTH-1:0] pipe5_b; reg pipe5_div0;
    // Stage 6 
    reg [A_WIDTH+B_WIDTH-1:0] pipe6_work; reg [B_WIDTH-1:0] pipe6_b; reg pipe6_div0;
    // Stage 7
    reg [A_WIDTH+B_WIDTH-1:0] pipe7_work; reg [B_WIDTH-1:0] pipe7_b; reg pipe7_div0;
    // Stage 8 
    reg [A_WIDTH+B_WIDTH-1:0] pipe8_work; reg [B_WIDTH-1:0] pipe8_b; reg pipe8_div0;
    // Stage 9
    reg [A_WIDTH+B_WIDTH-1:0] pipe9_work; reg [B_WIDTH-1:0] pipe9_b; reg pipe9_div0;
    // Stage 10 
    reg [A_WIDTH+B_WIDTH-1:0] pipe10_work; reg [B_WIDTH-1:0] pipe10_b; reg pipe10_div0;
    // Stage 11 
    reg [A_WIDTH+B_WIDTH-1:0] pipe11_work; reg [B_WIDTH-1:0] pipe11_b; reg pipe11_div0;

    // Combinational Logic 

    reg [A_WIDTH+B_WIDTH-1:0] next_work_s1;
    reg [A_WIDTH+B_WIDTH-1:0] next_work_s2;
    reg [A_WIDTH+B_WIDTH-1:0] next_work_s3;
    reg [A_WIDTH+B_WIDTH-1:0] next_work_s4;
    reg [A_WIDTH+B_WIDTH-1:0] next_work_s5;
    reg [A_WIDTH+B_WIDTH-1:0] next_work_s6;
    reg [A_WIDTH+B_WIDTH-1:0] next_work_s7;
    reg [A_WIDTH+B_WIDTH-1:0] next_work_s8;
    reg [A_WIDTH+B_WIDTH-1:0] next_work_s9;
    reg [A_WIDTH+B_WIDTH-1:0] next_work_s10;
    reg [A_WIDTH+B_WIDTH-1:0] next_work_s11;
    reg [A_WIDTH+B_WIDTH-1:0] next_work_s12; 

    integer i;

    // Computational Logic (12 Stages)

    always @(*) begin
        // Stage 1
        next_work_s1 = {{B_WIDTH{1'b0}}, a};
        for (i=0; i<STAGE_BITS; i=i+1) begin
            next_work_s1 = next_work_s1 << 1;
            if (next_work_s1[A_WIDTH+B_WIDTH-1 : A_WIDTH] >= b) begin
                next_work_s1[A_WIDTH+B_WIDTH-1 : A_WIDTH] = next_work_s1[A_WIDTH+B_WIDTH-1 : A_WIDTH] - b;
                next_work_s1[0] = 1'b1;
            end
        end

        // Stage 2
        next_work_s2 = pipe1_work;
        for (i=0; i<STAGE_BITS; i=i+1) begin
            next_work_s2 = next_work_s2 << 1;
            if (next_work_s2[A_WIDTH+B_WIDTH-1 : A_WIDTH] >= pipe1_b) begin
                next_work_s2[A_WIDTH+B_WIDTH-1 : A_WIDTH] = next_work_s2[A_WIDTH+B_WIDTH-1 : A_WIDTH] - pipe1_b;
                next_work_s2[0] = 1'b1;
            end
        end

        // Stage 3
        next_work_s3 = pipe2_work;
        for (i=0; i<STAGE_BITS; i=i+1) begin
            next_work_s3 = next_work_s3 << 1;
            if (next_work_s3[A_WIDTH+B_WIDTH-1 : A_WIDTH] >= pipe2_b) begin
                next_work_s3[A_WIDTH+B_WIDTH-1 : A_WIDTH] = next_work_s3[A_WIDTH+B_WIDTH-1 : A_WIDTH] - pipe2_b;
                next_work_s3[0] = 1'b1;
            end
        end

        // Stage 4
        next_work_s4 = pipe3_work;
        for (i=0; i<STAGE_BITS; i=i+1) begin
            next_work_s4 = next_work_s4 << 1;
            if (next_work_s4[A_WIDTH+B_WIDTH-1 : A_WIDTH] >= pipe3_b) begin
                next_work_s4[A_WIDTH+B_WIDTH-1 : A_WIDTH] = next_work_s4[A_WIDTH+B_WIDTH-1 : A_WIDTH] - pipe3_b;
                next_work_s4[0] = 1'b1;
            end
        end

        // Stage 5
        next_work_s5 = pipe4_work;
        for (i=0; i<STAGE_BITS; i=i+1) begin
            next_work_s5 = next_work_s5 << 1;
            if (next_work_s5[A_WIDTH+B_WIDTH-1 : A_WIDTH] >= pipe4_b) begin
                next_work_s5[A_WIDTH+B_WIDTH-1 : A_WIDTH] = next_work_s5[A_WIDTH+B_WIDTH-1 : A_WIDTH] - pipe4_b;
                next_work_s5[0] = 1'b1;
            end
        end

        // Stage 6
        next_work_s6 = pipe5_work;
        for (i=0; i<STAGE_BITS; i=i+1) begin
            next_work_s6 = next_work_s6 << 1;
            if (next_work_s6[A_WIDTH+B_WIDTH-1 : A_WIDTH] >= pipe5_b) begin
                next_work_s6[A_WIDTH+B_WIDTH-1 : A_WIDTH] = next_work_s6[A_WIDTH+B_WIDTH-1 : A_WIDTH] - pipe5_b;
                next_work_s6[0] = 1'b1;
            end
        end

        // Stage 7
        next_work_s7 = pipe6_work;
        for (i=0; i<STAGE_BITS; i=i+1) begin
            next_work_s7 = next_work_s7 << 1;
            if (next_work_s7[A_WIDTH+B_WIDTH-1 : A_WIDTH] >= pipe6_b) begin
                next_work_s7[A_WIDTH+B_WIDTH-1 : A_WIDTH] = next_work_s7[A_WIDTH+B_WIDTH-1 : A_WIDTH] - pipe6_b;
                next_work_s7[0] = 1'b1;
            end
        end

        // Stage 8
        next_work_s8 = pipe7_work;
        for (i=0; i<STAGE_BITS; i=i+1) begin
            next_work_s8 = next_work_s8 << 1;
            if (next_work_s8[A_WIDTH+B_WIDTH-1 : A_WIDTH] >= pipe7_b) begin
                next_work_s8[A_WIDTH+B_WIDTH-1 : A_WIDTH] = next_work_s8[A_WIDTH+B_WIDTH-1 : A_WIDTH] - pipe7_b;
                next_work_s8[0] = 1'b1;
            end
        end

        // Stage 9
        next_work_s9 = pipe8_work;
        for (i=0; i<STAGE_BITS; i=i+1) begin
            next_work_s9 = next_work_s9 << 1;
            if (next_work_s9[A_WIDTH+B_WIDTH-1 : A_WIDTH] >= pipe8_b) begin
                next_work_s9[A_WIDTH+B_WIDTH-1 : A_WIDTH] = next_work_s9[A_WIDTH+B_WIDTH-1 : A_WIDTH] - pipe8_b;
                next_work_s9[0] = 1'b1;
            end
        end

        // Stage 10
        next_work_s10 = pipe9_work;
        for (i=0; i<STAGE_BITS; i=i+1) begin
            next_work_s10 = next_work_s10 << 1;
            if (next_work_s10[A_WIDTH+B_WIDTH-1 : A_WIDTH] >= pipe9_b) begin
                next_work_s10[A_WIDTH+B_WIDTH-1 : A_WIDTH] = next_work_s10[A_WIDTH+B_WIDTH-1 : A_WIDTH] - pipe9_b;
                next_work_s10[0] = 1'b1;
            end
        end

        // Stage 11
        next_work_s11 = pipe10_work;
        for (i=0; i<STAGE_BITS; i=i+1) begin
            next_work_s11 = next_work_s11 << 1;
            if (next_work_s11[A_WIDTH+B_WIDTH-1 : A_WIDTH] >= pipe10_b) begin
                next_work_s11[A_WIDTH+B_WIDTH-1 : A_WIDTH] = next_work_s11[A_WIDTH+B_WIDTH-1 : A_WIDTH] - pipe10_b;
                next_work_s11[0] = 1'b1;
            end
        end

        // Stage 12
        next_work_s12 = pipe11_work;
        for (i=0; i<STAGE_BITS; i=i+1) begin
            next_work_s12 = next_work_s12 << 1;
            if (next_work_s12[A_WIDTH+B_WIDTH-1 : A_WIDTH] >= pipe11_b) begin
                next_work_s12[A_WIDTH+B_WIDTH-1 : A_WIDTH] = next_work_s12[A_WIDTH+B_WIDTH-1 : A_WIDTH] - pipe11_b;
                next_work_s12[0] = 1'b1;
            end
        end
    end


    // Sequential Logic 

    always @(posedge clk) begin
        if (!srst_n) begin
            // Reset all pipes
            pipe1_work <= 0; pipe1_b <= 0; pipe1_div0 <= 0;
            pipe2_work <= 0; pipe2_b <= 0; pipe2_div0 <= 0;
            pipe3_work <= 0; pipe3_b <= 0; pipe3_div0 <= 0;
            pipe4_work <= 0; pipe4_b <= 0; pipe4_div0 <= 0;
            pipe5_work <= 0; pipe5_b <= 0; pipe5_div0 <= 0;
            pipe6_work <= 0; pipe6_b <= 0; pipe6_div0 <= 0;
            pipe7_work <= 0; pipe7_b <= 0; pipe7_div0 <= 0;
            pipe8_work <= 0; pipe8_b <= 0; pipe8_div0 <= 0;
            pipe9_work <= 0; pipe9_b <= 0; pipe9_div0 <= 0;
            pipe10_work<= 0; pipe10_b<= 0; pipe10_div0<= 0;
            pipe11_work<= 0; pipe11_b<= 0; pipe11_div0<= 0;
        end else begin
            // Shift pipeline
            pipe1_work <= next_work_s1;  pipe1_b  <= b;          pipe1_div0  <= (b == 0);
            pipe2_work <= next_work_s2;  pipe2_b  <= pipe1_b;    pipe2_div0  <= pipe1_div0;
            pipe3_work <= next_work_s3;  pipe3_b  <= pipe2_b;    pipe3_div0  <= pipe2_div0;
            pipe4_work <= next_work_s4;  pipe4_b  <= pipe3_b;    pipe4_div0  <= pipe3_div0;
            pipe5_work <= next_work_s5;  pipe5_b  <= pipe4_b;    pipe5_div0  <= pipe4_div0;
            pipe6_work <= next_work_s6;  pipe6_b  <= pipe5_b;    pipe6_div0  <= pipe5_div0;
            pipe7_work <= next_work_s7;  pipe7_b  <= pipe6_b;    pipe7_div0  <= pipe6_div0;
            pipe8_work <= next_work_s8;  pipe8_b  <= pipe7_b;    pipe8_div0  <= pipe7_div0;
            pipe9_work <= next_work_s9;  pipe9_b  <= pipe8_b;    pipe9_div0  <= pipe8_div0;
            pipe10_work<= next_work_s10; pipe10_b <= pipe9_b;    pipe10_div0 <= pipe9_div0;
            pipe11_work<= next_work_s11; pipe11_b <= pipe10_b;   pipe11_div0 <= pipe10_div0;
        end
    end


    // Output Value

    assign quotient = next_work_s12[A_WIDTH-1:0];
    assign divide_by_0 = pipe11_div0;

endmodule