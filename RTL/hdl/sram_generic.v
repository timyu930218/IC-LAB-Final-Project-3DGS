module sram_generic #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 10,
    parameter DEPTH = 1024
) (
    input clk,
    input [DATA_WIDTH/8-1:0] wordmask, // Optional, can be ignored if not needed
    input csb,  // Chip Select (Active Low)
    input wsb,  // Write Enable (Active Low)
    input [DATA_WIDTH-1:0] wdata,
    input [ADDR_WIDTH-1:0] waddr,
    input [ADDR_WIDTH-1:0] raddr,
    output reg [DATA_WIDTH-1:0] rdata
);

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [DATA_WIDTH-1:0] _rdata;

    // Write
    integer i;
    always @(posedge clk) begin
        if (~csb && ~wsb) begin
            for (i=0; i<(DATA_WIDTH/8); i=i+1) begin
                if (wordmask[i] == 1'b0) begin
                    mem[waddr][i*8 +: 8] <= wdata[i*8 +: 8];
                end
            end
        end
    end

    // Read
    always @(posedge clk) begin
        if (~csb) begin
            _rdata <= mem[raddr];
        end
    end

    always @* begin
        rdata = #(1) _rdata;
    end

    // Backdoor Load Task
    task load_param(
        input integer index,
        input [DATA_WIDTH-1:0] val
    );
        mem[index] = val;
    endtask
    
endmodule
