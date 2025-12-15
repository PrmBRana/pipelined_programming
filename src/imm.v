`default_nettype none
module imm (
    input  wire [1:0]  ImmSrc,      // Immediate type selector (00: I, 01: S, 10: B, 11: J/U)
    input  wire [31:0] instruction, // Instruction from IF_ID_stage
    output reg  [31:0] ImmExt       // Sign-extended immediate
);
    always @(*) begin
        case (ImmSrc)
            2'b00: begin
                // I-type (addi, load, jalr)
                ImmExt = {{20{instruction[31]}}, instruction[31:20]};
            end
            2'b01: begin
                // S-type (store)
                ImmExt = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            end
            2'b10: begin
                // B-type (branch)
                ImmExt = {{19{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            end
            2'b11: begin
                // J-type (jal)
                ImmExt = {{11{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
                // If you want U-type instead:
                // ImmExt = {instruction[31:12], 12'b0};
            end
            default: ImmExt = 32'b0; // Invalid ImmSrc
        endcase
    end
endmodule
