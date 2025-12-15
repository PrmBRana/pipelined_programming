`timescale 1ns/1ps
`default_nettype none

module uart_bootloader (
    input  wire       clk,
    input  wire       reset,
    input  wire [7:0] rx_data,
    input  wire       rx_valid,

    output reg  [7:0] tx_data,
    output reg        tx_start,

    output reg        mem_we,
    output reg [7:0]  mem_addr,
    output reg [31:0] mem_wdata,
    output reg        stall_pro
);

    // ==========================================
    // CONSTANTS
    // ==========================================
    localparam HANDSHAKE_BYTE = 8'h25;
    localparam ACK  = 8'h55;
    localparam NACK = 8'hFF;

    // ==========================================
    // INTERNAL SIGNALS
    // ==========================================
    reg handshake_done;

    reg [1:0]  byte_count;          // 0..3
    reg [31:0] buffer0, buffer1;
    reg buffer_full0, buffer_full1;
    reg buffer_sel;

    reg [7:0] addr_count;

    // boot completion flag (set when zero sentinel written)
    reg boot_done;

    // detect rising edge of rx_valid
    reg rx_valid_d;
    wire rx_strobe = rx_valid & ~rx_valid_d;
    always @(posedge clk or posedge reset)
        if (reset) rx_valid_d <= 1'b0;
        else       rx_valid_d <= rx_valid;

    // ==========================================
    // MAIN LOGIC
    // ==========================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            tx_data        <= 8'd0;
            tx_start       <= 1'b0;
            mem_we         <= 1'b0;
            mem_addr       <= 8'd0;
            mem_wdata      <= 32'd0;

            handshake_done <= 1'b0;

            buffer0        <= 32'd0;
            buffer1        <= 32'd0;
            buffer_full0   <= 1'b0;
            buffer_full1   <= 1'b0;
            buffer_sel     <= 1'b0;
            byte_count     <= 2'd0;
            addr_count     <= 8'd0;

            stall_pro      <= 1'b1;   // hold CPU at reset
            boot_done      <= 1'b0;
        end else begin

            // keep mem_addr/mem_wdata stable unless writing
            // Stall default is to follow boot_done (if not done -> 1)
            stall_pro <= ~boot_done;

            // ===================================================
            // HANDSHAKE (single-byte handshake before sending code)
            // ===================================================
            if (!handshake_done && rx_strobe) begin
                if (rx_data == HANDSHAKE_BYTE) begin
                    tx_data        <= ACK;
                    tx_start       <= 1'b1;
                    handshake_done <= 1'b1;
                    byte_count     <= 2'd0;
                    // stall_pro remains asserted; boot_done still 0
                end else begin
                    tx_data  <= NACK;
                    tx_start <= 1'b1;
                end
            end

            // ===================================================
            // BUFFER FILLING (after handshake)
            // ===================================================
            else if (handshake_done && rx_strobe && !boot_done) begin
                // put received byte into selected buffer
                if (buffer_sel == 1'b0 && !buffer_full0) begin
                    case (byte_count)
                        2'd0: buffer0[7:0]   <= rx_data;
                        2'd1: buffer0[15:8]  <= rx_data;
                        2'd2: buffer0[23:16] <= rx_data;
                        2'd3: buffer0[31:24] <= rx_data;
                    endcase

                    if (byte_count == 2'd3)
                        buffer_full0 <= 1'b1;
                end
                else if (buffer_sel == 1'b1 && !buffer_full1) begin
                    case (byte_count)
                        2'd0: buffer1[7:0]   <= rx_data;
                        2'd1: buffer1[15:8]  <= rx_data;
                        2'd2: buffer1[23:16] <= rx_data;
                        2'd3: buffer1[31:24] <= rx_data;
                    endcase

                    if (byte_count == 2'd3)
                        buffer_full1 <= 1'b1;
                end

                // update byte counter and switch buffer on word completion
                if (byte_count == 2'd3) begin
                    byte_count <= 2'd0;
                    buffer_sel <= ~buffer_sel;
                end else begin
                    byte_count <= byte_count + 1'b1;
                end
            end

            // ===================================================
            // MEMORY WRITE (consume full buffers)
            // ===================================================
            // Only perform writes while boot not done.
            if (!boot_done) begin
                if (buffer_full0) begin
                    // issue a write of buffer0
                    mem_wdata <= buffer0;
                    mem_addr  <= addr_count;
                    mem_we    <= 1'b1;
                    addr_count <= addr_count + 1'b1;

                    // clear buffer after scheduling write
                    buffer0      <= 32'd0;
                    buffer_full0 <= 1'b0;

                    // If this word is sentinel zero, mark boot_done AFTER write
                    if (buffer0 == 32'd0) begin
                        boot_done <= 1'b1;
                        // stall_pro remains asserted this cycle; will be released next cycle
                    end
                end
                else if (buffer_full1) begin
                    mem_wdata <= buffer1;
                    mem_addr  <= addr_count;
                    mem_we    <= 1'b1;
                    addr_count <= addr_count + 1'b1;

                    buffer1      <= 32'd0;
                    buffer_full1 <= 1'b0;

                    if (buffer1 == 32'd0) begin
                        boot_done <= 1'b1;
                    end
                end
            end
            // if boot_done is set, stall_pro is released (above default)
        end
    end

endmodule
