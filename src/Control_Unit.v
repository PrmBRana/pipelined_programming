`default_nettype none

module Control (
    input  wire [6:0] Opcode,      // Instruction opcode
    input  wire [2:0] funct3,      // Function code
    input  wire [6:0] funct7,      // Function code for R-type
    output reg        RegWriteD,   // Register write enable
    output reg  [1:0] ResultSrcD,  // Writeback source (0: ALU, 1: memory, 2: PC+4)
    output reg        MemWriteD,   // Memory write enable
    output reg        jumpD,       // Jump enable
    output reg        jumpR,
    output reg        BranchD,     // Branch enable
    output reg  [3:0] ALUControlD, // ALU operation (4-bit)
    output reg        ALUSrcD,     // ALU second operand (0: rs2, 1: imm)
    output reg  [1:0] ImmSrc,      // Immediate type (00: I/U, 01: S, 10: B, 11: J)
    output reg  [2:0] FUN3,        // Pass-through funct3 (for load/store width/sign)
    output reg  [1:0] ALUType      // ALU instruction type: 00=R/I, 01=S, 10=B, 11=J
);

    always @(*) begin
        // Default values
        RegWriteD   = 1'b0;
        ResultSrcD  = 2'b00;
        MemWriteD   = 1'b0;
        jumpD       = 1'b0;
        jumpR       = 1'b0;
        BranchD     = 1'b0;
        ALUControlD = 4'b0000;
        ALUSrcD     = 1'b0;
        ImmSrc      = 2'b00;
        FUN3        = funct3;
        ALUType     = 2'b00;

        case (Opcode)
            // R-type (register-register)
            7'b0110011: begin
                RegWriteD = 1'b1;
                ALUSrcD   = 1'b0; // second operand from rs2
                ALUType   = 2'b00;
                case ({funct7, funct3})
                    {7'b0000000, 3'b000}: ALUControlD = 4'b0010; // ADD
                    {7'b0100000, 3'b000}: ALUControlD = 4'b0011; // SUB
                    {7'b0000000, 3'b110}: ALUControlD = 4'b0001; // OR
                    {7'b0000000, 3'b111}: ALUControlD = 4'b0000; // AND
                    {7'b0000000, 3'b100}: ALUControlD = 4'b0100; // XOR
                    {7'b0000000, 3'b001}: ALUControlD = 4'b0101; // SLL
                    {7'b0000000, 3'b101}: ALUControlD = 4'b0110; // SRL
                    {7'b0100000, 3'b101}: ALUControlD = 4'b0111; // SRA
                    {7'b0000000, 3'b010}: ALUControlD = 4'b1000; // SLT
                    {7'b0000000, 3'b011}: ALUControlD = 4'b1001; // SLTU
                    default: ALUControlD = 4'b0000; // default safe
                endcase
            end

            // I-type (ALU immediate)
            7'b0010011: begin
                RegWriteD = 1'b1;
                ALUSrcD   = 1'b1; // immediate as second operand
                ImmSrc    = 2'b00;
                ALUType   = 2'b00;
                case (funct3)
                    3'b000: ALUControlD = 4'b0010; // ADDI
                    3'b100: ALUControlD = 4'b0100; // XORI
                    3'b110: ALUControlD = 4'b0001; // ORI
                    3'b111: ALUControlD = 4'b0000; // ANDI
                    3'b001: ALUControlD = 4'b0101; // SLLI
                    3'b101: ALUControlD = (funct7 == 7'b0100000) ? 4'b0111 : 4'b0110; // SRAI : SRLI
                    3'b010: ALUControlD = 4'b1000; // SLTI
                    3'b011: ALUControlD = 4'b1001; // SLTIU
                    default: ALUControlD = 4'b0000;
                endcase
            end

            // LOAD (LB, LH, LW, LBU, LHU)
            7'b0000011: begin
                RegWriteD   = 1'b1;
                ResultSrcD  = 2'b01; // data from memory
                ALUSrcD     = 1'b1;  // address = rs1 + imm
                ImmSrc      = 2'b00; // I-type immediate
                ALUControlD = 4'b0010; // ADD for effective address
                ALUType     = 2'b00; // I-type
                // FUN3 tells memory stage which load (LB/LH/LW/LBU/LHU)
            end

            // STORE (SB, SH, SW)
            7'b0100011: begin
                MemWriteD   = 1'b1;
                ALUSrcD     = 1'b1;  // address = rs1 + imm
                ImmSrc      = 2'b01; // S-type immediate
                ALUControlD = 4'b0010; // ADD for effective address
                ALUType     = 2'b01; // S-type
                // FUN3 tells memory stage which store (SB/SH/SW)
            end

            // BRANCH (BEQ, BNE, BLT, BGE, BLTU, BGEU)
            7'b1100011: begin
                BranchD     = 1'b1;
                ALUSrcD     = 1'b0; // compare rs1 vs rs2
                ImmSrc      = 2'b10; // B-type immediate
                ALUControlD = 4'b0011; // SUB for comparison
                ALUType     = 2'b10;   // B-type
                // FUN3 indicates branch type
            end

            // JAL (J-type)
            7'b1101111: begin
                RegWriteD   = 1'b1;
                ResultSrcD  = 2'b10; // PC+4
                jumpD       = 1'b1;
                jumpR       = 1'b0;
                ImmSrc      = 2'b11; // J-type immediate
                ALUSrcD     = 1'b1;  // compute target as PC+imm
                ALUControlD = 4'b0010; // ADD
                ALUType     = 2'b11;   // J-type
            end

            // JALR (I-type)
            7'b1100111: begin
                RegWriteD   = 1'b1;
                ResultSrcD  = 2'b10; // PC+4
                jumpD       = 1'b1;
                jumpR       = 1'b1;
                ALUSrcD     = 1'b1;  // rs1 + imm
                ImmSrc      = 2'b00; // I-type immediate
                ALUControlD = 4'b0010; // ADD
                ALUType     = 2'b11;   // J-type
            end

            default: begin
                // Keep defaults (safe)
                RegWriteD   = 1'b0;
                ResultSrcD  = 2'b00;
                MemWriteD   = 1'b0;
                jumpD       = 1'b0;
                jumpR       = 1'b0;
                BranchD     = 1'b0;
                ALUControlD = 4'b0000;
                ALUSrcD     = 1'b0;
                ImmSrc      = 2'b00;
                FUN3        = funct3;
                ALUType     = 2'b00;
            end
        endcase
    end
endmodule
