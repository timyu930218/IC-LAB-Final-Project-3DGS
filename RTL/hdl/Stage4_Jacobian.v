module Stage4_Jacobian #(
    parameter BIT_WIDTH = 16, 
    parameter FLOAT_WIDTH = 6,
    parameter PRODUCT_WIDTH = 32,
    parameter RENDER_WIDTH = 128
) (
    input clk,
    input srst_n,
    input enable, 
    input [9*PRODUCT_WIDTH-1:0] jacobian_product,
    input [35:0] jacobian_quotient,

    output valid,

    input [3*BIT_WIDTH-1:0] campos, 
    output reg [6*BIT_WIDTH-1:0] jacobian,

    output reg signed [9*BIT_WIDTH-1:0] jacobian_multiplicand,
    output reg signed [9*BIT_WIDTH-1:0] jacobian_multiplier,

    output reg [35:0] jacobian_dividend,
    output reg [35:0] jacobian_divisor

);

    //========================
    // DECLARE
    //========================

    localparam  IDLE = 3'b000, 
                READ = 3'b001, 
                COMPUTE = 3'b010, 
                DONE = 3'b011;
    reg [2:0] state, state_n;
    reg [6*BIT_WIDTH-1:0] jacobian_n;

    //========================
    // DATAPATH
    //========================

    // read input from sram 
    wire signed [BIT_WIDTH-1:0] campos_x;
    wire signed [BIT_WIDTH-1:0] campos_y;
    wire signed [BIT_WIDTH-1:0] campos_z;
    assign campos_x = campos[3*BIT_WIDTH-1:2*BIT_WIDTH];
    assign campos_y = campos[2*BIT_WIDTH-1:BIT_WIDTH];
    assign campos_z = campos[BIT_WIDTH-1:0];

    // Constants Q10.6
    // fx = (RENDER_WIDTH / 64) * 3552
    // If 64: 3552 ('h0DE0)
    // If 128: 7104 ('h1BC0)
    wire signed [BIT_WIDTH-1:0] fx = (RENDER_WIDTH == 128) ? 16'd7104 : 16'd3552;
    wire signed [BIT_WIDTH-1:0] fy = (RENDER_WIDTH == 128) ? 16'd7104 : 16'd3552;

    wire signed [2*BIT_WIDTH-1:0] fx_shift = (RENDER_WIDTH == 128) ? (16'd7104 <<< 6) : (16'd3552 <<< 6);
    wire signed [2*BIT_WIDTH-1:0] fy_shift = (RENDER_WIDTH == 128) ? (16'd7104 <<< 6) : (16'd3552 <<< 6);
    
    // Extend to 36 bits to prevent overflow during intermediate calculations
    // max val approx 32768 * 32768 = 10^9, fits in 30 bits. 36 is safe.
    wire signed [35:0] z_ext = campos_z;   //z
    //wire signed [35:0] z_sq = campos_z * campos_z;  //z^2  //I change z_ext to campos_z

    wire  [BIT_WIDTH-1:0] j00;
    wire  [BIT_WIDTH-1:0] j01;
    wire  [BIT_WIDTH-1:0] j02;
    wire  [BIT_WIDTH-1:0] j10;
    wire  [BIT_WIDTH-1:0] j11;
    wire  [BIT_WIDTH-1:0] j12;
    
    // Rounding Logic: Round Towards Zero (Truncation)
    // Matches new golden data: sign * (abs(num) / den)
    // No rounding up.
    
    // j00 = fx / z (Always positive)
    reg signed [35:0] num_j00;
    reg signed [35:0] num_j00_n;
    reg signed [35:0] abs_num_j00_n; // fx is positive
    reg signed [35:0] abs_num_j00; // fx is positive
    reg signed [35:0] quot_j00_n;
    reg signed [35:0] quot_j00;
    assign j00 = quot_j00;

    assign j01 = 0;

    // j02 = (-fx * x) / z^2
    reg signed [35:0] num_j02;
    reg signed [35:0] num_j02_n;
    reg signed [35:0] abs_num_j02_n;
    reg signed [35:0] abs_num_j02;
    reg signed [35:0] quot_j02_n;
    reg signed [35:0] quot_j02;
    assign j02 = (num_j02 < 0) ? -quot_j02 : quot_j02;

    assign j10 = 0;

    // j11 = -fy / z (Always negative)
    reg signed [35:0] num_j11;
    reg signed [35:0] num_j11_n;
    reg signed [35:0] abs_num_j11_n; // -(-fy) = fy
    reg signed [35:0] abs_num_j11; // -(-fy) = fy
    reg signed [35:0] quot_j11_n;
    reg signed [35:0] quot_j11;
    assign j11 = -quot_j11;

    // j12 = (fy * y) / z^2
    reg signed [35:0] num_j12;
    reg signed [35:0] num_j12_n;
    reg signed [35:0] abs_num_j12_n;
    reg signed [35:0] abs_num_j12;
    reg signed [35:0] quot_j12_n;
    reg signed [35:0] quot_j12;
    assign j12 = (num_j12 < 0) ? -quot_j12 : quot_j12;

    reg [35:0] j02_den_n, j02_den;
    reg [35:0] j12_den_n, j12_den;

    reg [4:0] compute_count, compute_count_n;
    //========================
    // FSM(control unit)
    //========================

    always@(posedge clk)begin 
        if(!srst_n) begin 
            state <= IDLE;
            jacobian <= 0;
            compute_count <= 0;
        end else begin 
            state <= state_n;
            jacobian <= jacobian_n;
            compute_count <= compute_count_n;
            num_j00 <= num_j00_n;
            num_j02 <= num_j02_n;
            num_j11 <= num_j11_n;
            num_j12 <= num_j12_n;

            abs_num_j00 <= abs_num_j00_n;
            abs_num_j02 <= abs_num_j02_n;
            abs_num_j11 <= abs_num_j11_n;
            abs_num_j12 <= abs_num_j12_n;
            quot_j00 <= quot_j00_n;
            quot_j02 <= quot_j02_n;
            quot_j11 <= quot_j11_n;
            quot_j12 <= quot_j12_n;

            j02_den <= j02_den_n;
            j12_den <= j12_den_n;

        end
    end

    always @* begin
        jacobian_multiplicand = 0;
        jacobian_multiplier = 0;
        case(compute_count)
            0: begin
                jacobian_multiplicand = {(-fx), fy, campos_z, 96'b0};
                jacobian_multiplier = {campos_x, campos_y, campos_z, 96'b0};
            end
            default:;
        endcase
    end

    always@(*)begin 
        state_n = state;
        jacobian_n = jacobian;
        compute_count_n = compute_count;

        num_j00_n = num_j00;
        num_j02_n = num_j02;
        num_j11_n = num_j11;
        num_j12_n = num_j12;

        abs_num_j00_n = abs_num_j00;
        abs_num_j02_n = abs_num_j02;
        abs_num_j11_n = abs_num_j11;
        abs_num_j12_n = abs_num_j12;

        quot_j00_n = quot_j00;
        quot_j02_n = quot_j02;
        quot_j11_n = quot_j11;
        quot_j12_n = quot_j12;

        jacobian_dividend = 0;
        jacobian_divisor = 0;

        j02_den_n = j02_den;
        j12_den_n = j12_den;

        case(state)
            IDLE: begin
                if(enable) begin
                    state_n = COMPUTE;
                end
            end
            COMPUTE: begin
                // Single cycle compute for now
                case (compute_count)
                    0: begin
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    1: begin
                        num_j00_n = fx_shift;
                        abs_num_j00_n = num_j00_n; // fx is positive

                        num_j02_n = jacobian_product[9*PRODUCT_WIDTH-1:8*PRODUCT_WIDTH] <<< 6;
                        abs_num_j02_n = (num_j02_n < 0) ? -num_j02_n : num_j02_n;
                        j02_den_n = jacobian_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH];

                        num_j11_n = -fy_shift;
                        abs_num_j11_n = fy_shift;  // -(-fy) = fy

                        num_j12_n = jacobian_product[8*PRODUCT_WIDTH-1:7*PRODUCT_WIDTH] <<< 6;
                        abs_num_j12_n = (num_j12_n < 0) ? -num_j12_n : num_j12_n;
                        j12_den_n = jacobian_product[7*PRODUCT_WIDTH-1:6*PRODUCT_WIDTH];

                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    2: begin // j00 = fx / z (Always positive)
                        jacobian_dividend = abs_num_j00;
                        jacobian_divisor = z_ext;
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    3: begin // j02 = (-fx * x) / z^2
                        jacobian_dividend = abs_num_j02;
                        jacobian_divisor = j02_den;
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    4: begin // j11 = -fy / z (Always negative)
                        jacobian_dividend = abs_num_j11;
                        jacobian_divisor = z_ext;
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    5: begin // j12 = (fy * y) / z^2
                        jacobian_dividend = abs_num_j12;
                        jacobian_divisor = j12_den;
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    6, 7, 8, 9, 10, 11, 12: begin
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    13: begin
                        quot_j00_n = jacobian_quotient;
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    14: begin
                        quot_j02_n = jacobian_quotient;
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    15: begin
                        quot_j11_n = jacobian_quotient;
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    16: begin
                        quot_j12_n = jacobian_quotient;
                        compute_count_n = compute_count + 1;
                        state_n = COMPUTE;
                    end

                    17: begin
                        jacobian_n = {j00, j01, j02, j10, j11, j12};
                        compute_count_n = 0;
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