module ALU (
    input  wire [31:0] ScrA,
    input  wire [31:0] ScrB,
    input  wire [3:0]  ALUControl,  // From Control unit
    input  wire [1:0]  ALUType,     // 00: R/I, 01: S, 10: B, 11: J
    output reg  [31:0] ALUResult,
    output reg         Zero
);

always @(*) begin
    ALUResult = 32'd0;
    Zero      = 1'b0;

    case (ALUType)
        2'b00: begin // R/I-type
            case (ALUControl)
                4'b0010: ALUResult = ScrA + ScrB;                 // ADD/ADDI
                4'b0011: ALUResult = ScrA - ScrB;                 // SUB
                4'b0000: ALUResult = ScrA & ScrB;                 // AND
                4'b0001: ALUResult = ScrA | ScrB;                 // OR
                4'b0100: ALUResult = ScrA ^ ScrB;                 // XOR
                4'b0101: ALUResult = ScrA << ScrB[4:0];          // SLL
                4'b0110: ALUResult = ScrA >> ScrB[4:0];          // SRL
                4'b0111: ALUResult = $signed(ScrA) >>> ScrB[4:0]; // SRA
                4'b1000: ALUResult = ($signed(ScrA) < $signed(ScrB)) ? 32'd1 : 32'd0; // SLT/SLTI
                4'b1001: ALUResult = (ScrA < ScrB) ? 32'd1 : 32'd0;                   // SLTU/SLTIU
                default: ALUResult = 32'd0;
            endcase
        end
        2'b01: ALUResult = ScrA + ScrB; // S-type (store address)
        2'b10: begin                   // Branch
            case (ALUControl)
                4'b0000: Zero = (ScrA == ScrB);          // BEQ
                4'b0001: Zero = (ScrA != ScrB);          // BNE
                4'b0010: Zero = ($signed(ScrA) < $signed(ScrB)); // BLT
                4'b0011: Zero = ($signed(ScrA) >= $signed(ScrB));// BGE
                4'b0100: Zero = (ScrA < ScrB);           // BLTU
                4'b0101: Zero = (ScrA >= ScrB);          // BGEU
                default: Zero = 1'b0;
            endcase
        end
        2'b11: ALUResult = ScrA + ScrB; // J-type (PC+imm)
        default: ALUResult = 32'd0;
    endcase
end

endmodule
