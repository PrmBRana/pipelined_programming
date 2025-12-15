`default_nettype none
module Hazard_Unit(
    input  wire [4:0]  Rs1D,
    input  wire [4:0]  Rs2D,
    input  wire [4:0]  Rs1E,
    input  wire [4:0]  Rs2E,
    input  wire [4:0]  RdE,
    input  wire        RegWriteE,
    input  wire        PCSRCE,
    input  wire [1:0]  ResultSrcE_in,
    input  wire [4:0]  RdM,
    input  wire [4:0]  RdW,
    input  wire        RegWriteM,
    input  wire        RegWriteW,
    output reg         StallF,
    output reg         StallD,
    output reg         FlushD,
    output reg         FlushE,
    output reg [1:0]   Forward_AE,
    output reg [1:0]   Forward_BE
);
    always @(*) begin
            Forward_AE = 2'b00;
            Forward_BE = 2'b00;
            StallF     = 1'b0;
            StallD     = 1'b0;
            FlushD     = 1'b0;
            FlushE     = 1'b0;

            // Forwarding logic
            if ((Rs1E == RdM) && RegWriteM && (Rs1E != 0)) begin
                Forward_AE = 2'b10;
            end else if ((Rs1E == RdW) && RegWriteW && (Rs1E != 0)) begin
                Forward_AE = 2'b01;
            end

            if ((Rs2E == RdM) && RegWriteM && (Rs2E != 0)) begin
                Forward_BE = 2'b10;
            end else if ((Rs2E == RdW) && RegWriteW && (Rs2E != 0)) begin
                Forward_BE = 2'b01;
            end

            // Load-use hazard
            if ((ResultSrcE_in == 2'b01) && RegWriteE && ((Rs1D == RdE) || (Rs2D == RdE)) && (RdE != 0)) begin
                StallD = 1'b1;
                StallF = 1'b1;
            end

            // Branch/jump flush
            if (PCSRCE) begin
                FlushD = 1'b1;
                FlushE = 1'b1;
                StallF = 1'b0;
                StallD = 1'b0;
            end
        end
endmodule
