module fsm (
    input clk,
    input srst_n,
    input enable,
    input rot_valid,
    input conv_valid,
    input campos_valid,
    input jacobian_valid,
    input conv2d_valid,
    input conic_valid,
    input sort_valid,
    input render_valid,
    input [6:0] gaussian_num,
    output reg [3:0] state,
    output reg rot_enable,
    output reg conv_enable,
    output reg campos_enable, 
    output reg jacobian_enable, 
    output reg conv2d_enable,
    output reg conic_enable, 
    output reg sort_enable, 
    output reg render_enable, 
    output reg [6:0] gaussian_cnt
    
);

    // ========================
    // FSM
    // =========================
    localparam  IDLE = 4'b0000, 
                ROT  = 4'b0001, 
                COV3D = 4'b0010, 
                CAMPOS = 4'b0011, 
                JACOBIAN = 4'b0100, 
                COV2D = 4'b0101, 
                CONIC = 4'b0110, 
                SORT = 4'b0111, 
                RENDER = 4'b1000, 
                DONE = 4'b1001;

    reg [3:0] state_n;
    reg [6:0] gaussian_cnt_n ;

    // sequential circuit
    always @(posedge clk) begin
        if(srst_n == 0) begin
            state <= IDLE; 
            gaussian_cnt <= 0;
        end else begin
            state <= state_n;
            gaussian_cnt <= gaussian_cnt_n;
        end
    end

    // state transition
    always@(*)begin 
        state_n = state;
        rot_enable = 0;
        conv_enable = 0;
        campos_enable = 0;
        jacobian_enable = 0;
        conv2d_enable = 0;
        conic_enable = 0;
        sort_enable = 0;
        render_enable = 0;
        gaussian_cnt_n = gaussian_cnt;
        case(state)
            IDLE: begin
                if(enable) begin
                    state_n = ROT;
                    rot_enable = 1;
                    gaussian_cnt_n = 0;
                end
            end
            ROT: begin
                if(rot_valid) begin
                    state_n = COV3D;
                    conv_enable = 1;
                end
            end
            COV3D: begin
                if(conv_valid) begin
                    state_n = CAMPOS;
                    campos_enable = 1;
                end
            end
            CAMPOS: begin
                if(campos_valid) begin
                    state_n = JACOBIAN;
                    jacobian_enable = 1;
                end
            end
            JACOBIAN: begin
                if(jacobian_valid) begin
                    state_n = COV2D;
                    conv2d_enable = 1;
                end
            end
            COV2D: begin
                if(conv2d_valid) begin
                    state_n = CONIC;
                    conic_enable = 1;
                end
            end
            CONIC: begin
                // TODO : check gaussian number
                if(conic_valid && gaussian_cnt+1 >= gaussian_num) begin
                    state_n = SORT; 
                    sort_enable = 1;
                end else if(conic_valid) begin
                    gaussian_cnt_n = gaussian_cnt + 1;
                    state_n = ROT;
                    rot_enable = 1; 
                end 
            end
            SORT: begin
                if(sort_valid) begin
                    state_n = RENDER;
                    render_enable = 1;
                end
            end
            RENDER: begin 
                if(render_valid) begin
                    state_n = DONE;
                end
            end
            DONE : begin 
                state_n = IDLE;
            end
        endcase
    end



endmodule