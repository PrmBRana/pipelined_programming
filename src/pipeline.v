`default_nettype none
`timescale 1ns/1ps

module pipeline(
    input  wire clk,
    input  wire reset,
    input  wire rx,
    output wire tx
);
    // wires
    wire [31:0] PCPLUS4_top, PC_top, PCF, Instruction1_out, INSTRUCTION;
    wire [31:0] RD1_top, RD2_top, PCD_top, PCE_top, PCPLUS4D_TOP;
    wire [31:0] RD1E_top, RD2E_top, SrcA_top, outB_top, ScrB_top;
    wire [31:0] ALUResultE_top, PCPlus4E_top, ALUResultM_top, WriteDataM_top, PCPlus4M_top;
    wire [31:0] Datamem_top, ALUResultW_top, ReadDataW_top, PCPlus4W_top, ResultW_top, PCTarget_top;
    wire [31:0] ImmExtD_top, ImmExtE_top;

    wire RegWrite_top, ALUSrcD_top, memWriteD_top, jumpD_top, BranchD_top;
    wire JumpE_top, BranchE_top, zero_top, ANDout_top, PCSCR_top, jumpRD_top, JumpRE_top;
    wire RegWriteE_top, MemWriteE_top, ALUSrcE_top, MemWriteM_top, RegWriteM_top, RegWriteW_top;
    wire StallF_top, StallD_top, FlushD_top, FlushE_top;

    wire [1:0] ImmSrc_top, ResultSrcD_top, ALUtyp_top, ALUTypE_top, ResultSrcE_top, ResultSrcM_top, ResultSrcW_top, ForwardAE_top, ForwardBE_top;
    wire [3:0] ALUControlD_top, ALUControlE_top;
    wire [2:0] FUN3_top, FUN3E_top, FUN3M_top;
    wire [4:0] RdE_top, RdM_top, Rs1E_top, Rs2E_top, RdW_top;

        // ----------------- UART Signals -------------------
    wire [7:0] uart_rx_data;
    wire       uart_rx_ready;
    wire [7:0] boot_tx_data;
    wire       boot_tx_start;
    wire       uart_tx_busy;

    //-------------------Bootloader-----------------------
    wire        Write_enable;
    wire [7:0]  mem_addr;
    wire [31:0] mem_wdata;

    // stall produced by bootloader: asserted while loading
    wire stall_Pro;

    // combine hazard stalls with bootloader stall:
    // if either requests a stall, pipeline should freeze
    wire StallF_net = stall_Pro | StallF_top;
    wire StallD_net = stall_Pro | StallD_top;
    //-------------------------------Instruction Fetch--------------------
    // Program Counter Increment 
    PC_incre PC(
        .pc(PCF),
        .PCPlus4(PCPLUS4_top)
    );

    // Select PC and Target PC
    PCSelect_MUX PCSelect_top(
        .PCScr(PCSCR_top),
        .PCSequential(PCPLUS4_top),
        .PCBranch(PCTarget_top),
        .Mux3_PC(PC_top)
    );

    // Clocked PC register
    pc_register Register_top(
        .clk(clk),
        .reset(reset),
        .PCF_in(PC_top),
        .stallF(StallF_net),
        .PCF_out(PCF)
    );

   
    // ---------------- UART Loader and Instruction Memory -----------------
    // ==============================
    // UART module
    // ==============================
    uart_Tx_fixed #(
        .CLK_FREQ(50_000_000),
        .BAUD_RATE(115_200),
        .OVERSAMPLE(16)
    ) uart_inst (
        .clk(clk),
        .reset(reset),
        // Transmitter
        .tx_Start(boot_tx_start),
        .tx_Data(boot_tx_data),
        .tx(tx),
        .tx_busy(uart_tx_busy),
        // Receiver
        .rx(rx),
        .rx_Data(uart_rx_data),
        .rx_ready(uart_rx_ready)
    );

    // ==============================
    // Bootloader module
    // ==============================
    uart_bootloader uart_bootloader (
        .clk(clk),
        .reset(reset),
        .rx_data(uart_rx_data),
        .rx_valid(uart_rx_ready),
        .tx_data(boot_tx_data),
        .tx_start(boot_tx_start),
        .mem_we(Write_enable),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .stall_pro(stall_Pro)    
    );

    //===============================
    // 1KB 32-bit flipflop memory
    //===============================
    mem1KB_32bit flipflop(
        .clk(clk),
        .reset(reset),
        .we(Write_enable),
        .addr(mem_addr),    // connect to bootloader
        .wdata(mem_wdata),  // connect to bootloader
        .read_Address(PCF), // PCF provided as full 32-bit byte-address; mem1KB_32bit should index [9:2]
        .Instruction_out(Instruction1_out)   // optional, for reading
        
    );

    //----------------------------Instruction Decode------------------------------------
    IF_ID_stage IF_DF_top(
        .clk(clk),
        .reset(reset),
        .stallD(StallD_net),
        .flushD(FlushD_top),
        .PC_in(PCF),
        .PCplus4_in(PCPLUS4_top),
        .instruction_in(Instruction1_out),
        .instruction_out(INSTRUCTION),
        .PCplus4_out(PCPLUS4D_TOP),
        .PC_out(PCD_top)
    );

    // Control Unit
    Control control(
        .Opcode(INSTRUCTION[6:0]),
        .funct3(INSTRUCTION[14:12]),
        .funct7(INSTRUCTION[31:25]),
        .RegWriteD(RegWrite_top),
        .ResultSrcD(ResultSrcD_top),
        .MemWriteD(memWriteD_top),
        .jumpD(jumpD_top),
        .jumpR(jumpRD_top),
        .BranchD(BranchD_top),
        .ALUControlD(ALUControlD_top),
        .ALUSrcD(ALUSrcD_top),
        .ImmSrc(ImmSrc_top),
        .FUN3(FUN3_top),
        .ALUType(ALUtyp_top)
    );

    // Register File
    Reg_file Reg_file_top(
        .clk(clk),
        .reset(reset),
        .rs1_addr(INSTRUCTION[19:15]),
        .rs2_addr(INSTRUCTION[24:20]),
        .rd_addr(RdW_top),
        .Regwrite(RegWriteW_top),
        .Write_data(ResultW_top),
        .Read_data1(RD1_top),
        .Read_data2(RD2_top)
    );

    // Immediate
    imm imm_top(
        .ImmSrc(ImmSrc_top),
        .instruction(INSTRUCTION),
        .ImmExt(ImmExtD_top)
    );

    //-----------------------------Execute Stage-------------------------------------------------------
    EX_stage ex_stage(
        .clk(clk),
        .reset(reset),
        .flushE(FlushE_top),

        // Inputs from Decode stage
        .RD1D_in(RD1_top),
        .RD2D_in(RD2_top),
        .ImmExtD_in(ImmExtD_top),
        .PCPlus4D_in(PCPLUS4D_TOP),
        .PC_D_in(PCD_top),
        .Rs1D_in(INSTRUCTION[19:15]),
        .Rs2D_in(INSTRUCTION[24:20]),
        .RdD_in(INSTRUCTION[11:7]),
        .ALUControlD_in(ALUControlD_top),
        .ALUSrcD_in(ALUSrcD_top),
        .RegWriteD_in(RegWrite_top),
        .ResultSrcD_in(ResultSrcD_top),
        .MemWriteD_in(memWriteD_top),
        .BranchD_in(BranchD_top),
        .JumpD_in(jumpD_top),
        .JumpR_in(jumpRD_top),
        .FUN3_in(FUN3_top),
        .ALUType_in(ALUtyp_top),

        // Outputs to Execute stage
        .RD1E_out(RD1E_top),
        .RD2E_out(RD2E_top),
        .ImmExtD_out(ImmExtE_top),
        .PCPlus4D_out(PCPlus4E_top),
        .PC_D_out(PCE_top),
        .Rs1D_out(Rs1E_top),
        .Rs2D_out(Rs2E_top),
        .RdD_out(RdE_top),
        .ALUControlD_out(ALUControlE_top),
        .ALUSrcD_out(ALUSrcE_top),
        .RegWriteD_out(RegWriteE_top),
        .ResultSrcD_out(ResultSrcE_top),
        .MemWriteD_out(MemWriteE_top),
        .BranchD_out(BranchE_top),
        .JumpD_out(JumpE_top),
        .JumpR_out(JumpRE_top),
        .FUN3_out(FUN3E_top),
        .ALUType_out(ALUTypE_top)
    );

    // Execute Multiplexers
    MUX_A muxA(
        .RD1(RD1E_top),
        .resultW(ResultW_top),
        .ALUres(ALUResultM_top),
        .ForwardAE(ForwardAE_top),
        .ScrA(SrcA_top)
    );

    MUX_B muxB(
        .RD2(RD2E_top),
        .ResWrite(ResultW_top),
        .ALURes(ALUResultM_top),
        .ForwardBE(ForwardBE_top),
        .outB(outB_top)
    );

    MUX_SCRB ScrB(
        .rd2(outB_top),
        .ImmEx(ImmExtE_top),
        .ALUSCRE(ALUSrcE_top),
        .SCRB(ScrB_top)
    );

    // PC Target
    Adder adder(
        .pc_E(PCE_top),
        .rd1_E(RD1E_top),
        .imm_2(ImmExtE_top),
        .JumpR(JumpRE_top),
        .PCTarget(PCTarget_top)
    );

    // Branch gating and PC selection
    AND AND_gate(
        .zero(zero_top),
        .BranchE(BranchE_top),
        .AND_out(ANDout_top)
    );

    OR OR_gate(
        .AND_in(ANDout_top),
        .JumpE(JumpE_top),
        .PCSCR(PCSCR_top)
    );

    // ALU
    ALU alu(
        .ScrA(SrcA_top),
        .ScrB(ScrB_top),
        .ALUControl(ALUControlE_top),
        .ALUType(ALUTypE_top),
        .ALUResult(ALUResultE_top),
        .Zero(zero_top)
    );

    //----------------------------Memory Stage----------------------------------------------
    MEM_stage mem_stage(
        .clk(clk),
        .reset(reset),
        .ALUResult_in(ALUResultE_top),
        .WriteData_in(outB_top),
        .RdM_in(RdE_top),
        .PCPlus4M_in(PCPlus4E_top),
        .RegWriteM_in(RegWriteE_top),
        .ResultSrcM_in(ResultSrcE_top),
        .MemWriteM_in(MemWriteE_top),
        .FUN3_in(FUN3E_top),
        .ALUResult_out(ALUResultM_top),
        .WriteData_out(WriteDataM_top),
        .RdM_out(RdM_top),
        .PCPlus4M_out(PCPlus4M_top),
        .RegWriteM_out(RegWriteM_top),
        .ResultSrcM_out(ResultSrcM_top),
        .MemWriteM_out(MemWriteM_top),
        .FUN3_out(FUN3M_top)
    );

    // Data Memory
    DataMem datamem(
        .clk(clk),
        .reset(reset),
        .aluAddress_in(ALUResultM_top),
        .DataWriteM_in(WriteDataM_top),
        .memwriteM_in(MemWriteM_top),
        .func3(FUN3M_top),
        .DataMem_out(Datamem_top)
    );

    //--------------------------------Writeback stage--------------------------------------------
    WriteBack_stage writeback_stage(
        .clk(clk),
        .reset(reset),
        .ALUResultW_in(ALUResultM_top),
        .ReadDataW_in(Datamem_top),
        .RdW_in(RdM_top),
        .PCPlus4W_in(PCPlus4M_top),
        .RegWriteW_in(RegWriteM_top),
        .ResultSrcW_in(ResultSrcM_top),
        .ALUResultW_out(ALUResultW_top),
        .ReadDataW_out(ReadDataW_top),
        .RdW_out(RdW_top),
        .PCPlus4W_out(PCPlus4W_top),
        .RegWriteW_out(RegWriteW_top),
        .ResultSrcW_out(ResultSrcW_top)
    );

    // Result write control (select ALU/mem/PC+4)
    Write_back write_back(
        .ALUResultW_in(ALUResultW_top),
        .ReadDataW_in(ReadDataW_top),
        .PCPlus4W_in(PCPlus4W_top),
        .ResultSrcW_in(ResultSrcW_top),
        .ResultW(ResultW_top)
    );

    //---------------------Hazard Unit-------------------------------------------------
    Hazard_Unit hazard(
        .Rs1D(INSTRUCTION[19:15]),
        .Rs2D(INSTRUCTION[24:20]),
        .Rs1E(Rs1E_top),
        .Rs2E(Rs2E_top),
        .RdE(RdE_top),
        .RegWriteE(RegWriteE_top),
        .PCSRCE(PCSCR_top),
        .ResultSrcE_in(ResultSrcE_top),
        .RdM(RdM_top),
        .RdW(RdW_top),
        .RegWriteM(RegWriteM_top),
        .RegWriteW(RegWriteW_top),
        .StallF(StallF_top),
        .StallD(StallD_top),
        .FlushD(FlushD_top),
        .FlushE(FlushE_top),
        .Forward_AE(ForwardAE_top),
        .Forward_BE(ForwardBE_top)
    );
endmodule
