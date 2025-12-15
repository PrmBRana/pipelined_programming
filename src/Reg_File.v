`default_nettype none
module Reg_file (
    input  wire        clk,         // Clock signal
    input  wire        reset,      // Asynchronous active-high reset
    input  wire [4:0]  rs1_addr,   // Read address 1 (source register 1)
    input  wire [4:0]  rs2_addr,   // Read address 2 (source register 2)
    input  wire [4:0]  rd_addr,    // Write address (destination register)
    input  wire        Regwrite,   // Write enable
    input  wire [31:0] Write_data, // Data to write
    output wire [31:0] Read_data1, // Data from rs1
    output wire [31:0] Read_data2  // Data from rs2
);
    integer k;
    reg [31:0] Register [0:31];

    // Initialize registers after reset
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Clear all registers
            for (k = 0; k < 32; k = k + 1)
                Register[k] <= 32'd0;
        end
        else if (Regwrite && rd_addr != 5'd0) begin
            Register[rd_addr] <= Write_data; // Write to destination register
        end
    end

    // Combinational read ports
    assign Read_data1 = (rs1_addr == 5'd0) ? 32'd0 : Register[rs1_addr];
    assign Read_data2 = (rs2_addr == 5'd0) ? 32'd0 : Register[rs2_addr];

endmodule