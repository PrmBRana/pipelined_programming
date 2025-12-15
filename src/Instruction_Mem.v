`timescale 1ns/1ps
`default_nettype none

module mem1KB_32bit (
    input  wire        clk,
    input  wire        reset,
    input  wire        we,        // write enable
    input  wire [7:0]  addr,      // 0..255
    input  wire [31:0] wdata,     // write data
    input wire [31:0] read_Address,
    output wire [31:0] Instruction_out      // read data
);

    localparam DEPTH = 256; // 256 words × 32-bit = 1 KB
    reg [31:0] mem [0:DEPTH-1];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for(i = 0; i < DEPTH; i = i + 1)
                mem[i] <= 32'd0;
        end
        else if (we && addr < DEPTH) begin
            mem[addr] <= wdata;
        end
    end

    // Instruction read: use word-aligned PC → bits [9:2]
    assign Instruction_out = mem[read_Address[9:2]];

endmodule
