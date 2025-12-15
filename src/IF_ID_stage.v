`default_nettype none
module IF_ID_stage (
    input wire clk,              // Clock signal
    input wire reset,           // Asynchronous active-high reset
    input wire stallD,          // Stall signal for decode stage
    input wire flushD,          // Flush signal to insert NOP
    input wire [31:0] PC_in,    // Current PC from fetch stage
    input wire [31:0] PCplus4_in, // PC + 4 from fetch stage
    input wire [31:0] instruction_in, // Fetched instruction
    output reg [31:0] instruction_out, // Instruction for decode stage
    output reg [31:0] PCplus4_out,    // PC + 4 for decode stage
    output reg [31:0] PC_out         // Current PC for decode stage
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            instruction_out <= 32'b0; // Clear outputs on reset
            PCplus4_out <= 32'b0;
            PC_out <= 32'b0;
        end
        else if (flushD) begin
            instruction_out <= 32'b0; // Insert NOP on flush
            PCplus4_out <= PCplus4_out; // Retain PC values
            PC_out <= PC_out;
        end
        else if (!stallD) begin
            instruction_out <= instruction_in; // Normal pipeline advance
            PCplus4_out <= PCplus4_in;
            PC_out <= PC_in;
        end
        else begin
            instruction_out <= instruction_out; // Retain values during stall
            PCplus4_out <= PCplus4_out;
            PC_out <= PC_out;
        end
    end
endmodule