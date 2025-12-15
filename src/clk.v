`default_nettype none
module pc_register (
    input wire clk,              // Clock signal
    input wire reset,           // Asynchronous active-high reset
    input wire [31:0] PCF_in,   // Next program counter value
    input wire stallF,          // Stall signal for fetch stage
    output reg [31:0] PCF_out   // Current program counter value
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PCF_out <= 32'b0;   // Reset PC to 0
        end
        else if (!stallF) begin
            PCF_out <= PCF_in;  // Update PC when not stalled
        end
        else begin
            PCF_out <= PCF_out; // Explicitly retain PC during stall
        end
    end
endmodule