`default_nettype none
module MUX_A(
    input wire [31:0] RD1,
    input wire [31:0] resultW,
    input wire [31:0] ALUres,
    input wire [1:0] ForwardAE, 
    output wire [31:0] ScrA
);
    assign ScrA = (ForwardAE == 2'b00) ? RD1 : (ForwardAE == 2'b01) ? resultW : ALUres;
endmodule

module MUX_B(
    input wire [31:0] RD2,
    input wire [31:0] ResWrite,
    input wire [31:0] ALURes,
    input wire [1:0] ForwardBE,
    output wire [31:0] outB
);
    assign outB = (ForwardBE == 2'b00) ? RD2 : (ForwardBE == 2'b01) ? ResWrite : ALURes;
endmodule

module MUX_SCRB(
    input wire [31:0] rd2,
    input wire [31:0] ImmEx,
    input wire ALUSCRE,
    output wire [31:0] SCRB
);
    assign SCRB = (ALUSCRE == 1'b0) ? rd2 : ImmEx;
endmodule


module Adder(
    input wire [31:0] pc_E,      // PC in execute stage (for JAL, branches)
    input wire [31:0] rd1_E,     // RD1 in execute stage (for JALR)
    input wire [31:0] imm_2,     // Sign-extended immediate
    input wire JumpR,            // JALR indicator (1 for JALR, 0 for JAL/branches)
    output wire [31:0] PCTarget  // Computed target address
);

    wire [31:0] base_addr;
    assign base_addr = JumpR ? rd1_E : pc_E;

    // For JALR, clear LSB. For JAL/branches, addition is enough (PC is word-aligned).
    assign PCTarget = JumpR ? ((base_addr + imm_2) & 32'hFFFFFFFE)
                            : (base_addr + imm_2);

endmodule


module AND(
    input wire zero,
    input wire BranchE,
    output wire AND_out
);
    assign AND_out = zero & BranchE;
endmodule

module OR(
    input wire AND_in,
    input wire JumpE,
    output wire PCSCR
);
    assign PCSCR = AND_in | JumpE;
endmodule

`default_nettype wire
