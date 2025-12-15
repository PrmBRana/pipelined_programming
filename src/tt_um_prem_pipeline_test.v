`default_nettype none
`timescale 1ns/1ps

module tt_um_prem_pipeline_test (
    input  wire [7:0] ui_in,    // Dedicated inputs (unused for UART now)
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path
    input  wire       ena,      // always 1 when powered
    input  wire       clk,      // clock
    input  wire       rst_n    // reset_n
    );

    wire rst = !rst_n;
    assign uio_oe = 8'b0000_0000;
    assign uio_out = 8'b0000_0000;
    assign uo_out[7:1] = 7'b000_0000;

    // Instantiate Top module (pipeline + UART loader)
    pipeline Top_inst(
    .clk(clk),
    .reset(rst),
    .rx(ui_in[0]),
    .tx(uo_out[0])
    );
endmodule
