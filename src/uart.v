`default_nettype none
`timescale 1ns / 1ps

module uart_Tx_fixed #(
    parameter CLK_FREQ    = 50_000_000,  // 50 MHz
    parameter BAUD_RATE   = 115_200,
    parameter OVERSAMPLE  = 16
)(
    input  wire       clk,
    input  wire       reset,        // Active-high reset
    // Transmitter
    input  wire       tx_Start,
    input  wire [7:0] tx_Data,
    output reg        tx,
    output reg        tx_busy,
    // Receiver
    input  wire       rx,
    output reg  [7:0] rx_Data,
    output reg        rx_ready
);

    // ================================
    // BAUD RATE GENERATOR
    // ================================
    localparam integer BAUD_DIV = CLK_FREQ / (BAUD_RATE * OVERSAMPLE);
    localparam integer CNT_WIDTH = $clog2(BAUD_DIV);

    reg [CNT_WIDTH-1:0] baud_cnt = 0;
    reg                 baud_tick = 0;

    always @(posedge clk) begin
        if (reset) begin
            baud_cnt  <= 0;
            baud_tick <= 0;
        end else begin
            if (baud_cnt == BAUD_DIV - 1) begin
                baud_cnt  <= 0;
                baud_tick <= 1;
            end else begin
                baud_cnt  <= baud_cnt + 1;
                baud_tick <= 0;
            end
        end
    end

    // ================================
    // UART TRANSMITTER (FIXED)
    // ================================
    localparam [1:0] TX_IDLE  = 2'b00,
                     TX_START = 2'b01,
                     TX_DATA  = 2'b10,
                     TX_STOP  = 2'b11;

    reg [1:0]  tx_state = TX_IDLE;
    reg [7:0]  tx_shift_reg = 8'd0;
    reg [2:0]  tx_bit_cnt = 3'd0;
    reg [3:0]  tx_oversample_cnt = 4'd0;  // ← NEW: Counter for oversampling

    always @(posedge clk) begin
        if (reset) begin
            tx                 <= 1'b1;
            tx_busy            <= 1'b0;
            tx_state           <= TX_IDLE;
            tx_shift_reg       <= 8'd0;
            tx_bit_cnt         <= 3'd0;
            tx_oversample_cnt  <= 4'd0;
        end else if (baud_tick) begin
            case (tx_state)
                TX_IDLE: begin
                    tx                <= 1'b1;
                    tx_busy           <= 1'b0;
                    tx_oversample_cnt <= 4'd0;
                    if (tx_Start) begin
                        tx_shift_reg <= tx_Data;
                        tx_state     <= TX_START;
                        tx_busy      <= 1'b1;
                    end
                end

                TX_START: begin
                    tx <= 1'b0;  // Start bit
                    if (tx_oversample_cnt == (OVERSAMPLE - 1)) begin
                        tx_state          <= TX_DATA;
                        tx_bit_cnt        <= 3'd0;
                        tx_oversample_cnt <= 4'd0;
                    end else begin
                        tx_oversample_cnt <= tx_oversample_cnt + 1;
                    end
                end

                TX_DATA: begin
                    tx <= tx_shift_reg[tx_bit_cnt];
                    if (tx_oversample_cnt == (OVERSAMPLE - 1)) begin
                        tx_oversample_cnt <= 4'd0;
                        if (tx_bit_cnt == 3'd7) begin
                            tx_state <= TX_STOP;
                        end else begin
                            tx_bit_cnt <= tx_bit_cnt + 1;
                        end
                    end else begin
                        tx_oversample_cnt <= tx_oversample_cnt + 1;
                    end
                end

                TX_STOP: begin
                    tx <= 1'b1;  // Stop bit
                    if (tx_oversample_cnt == (OVERSAMPLE - 1)) begin
                        tx_busy           <= 1'b0;
                        tx_state          <= TX_IDLE;
                        tx_oversample_cnt <= 4'd0;
                    end else begin
                        tx_oversample_cnt <= tx_oversample_cnt + 1;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // ================================
    // UART RECEIVER (unchanged)
    // ================================
    localparam [1:0] RX_IDLE  = 2'b00,
                     RX_START = 2'b01,
                     RX_DATA  = 2'b10,
                     RX_STOP  = 2'b11;

    reg [1:0]  rx_state     = RX_IDLE;
    reg [7:0]  rx_shift_reg = 8'd0;
    reg [2:0]  rx_bit_cnt   = 3'd0;
    reg [3:0]  rx_sample_cnt = 4'd0;
    reg [4:0]  one_counts = 0;

    always @(posedge clk) begin
        if (reset) begin
            rx_state      <= RX_IDLE;
            rx_shift_reg  <= 8'd0;
            rx_bit_cnt    <= 3'd0;
            rx_sample_cnt <= 4'd0;
            one_counts    <= 5'd0;
            rx_Data       <= 8'd0;
            rx_ready      <= 1'b0;
        end else if (baud_tick) begin
            

            case (rx_state)
                RX_IDLE: begin
                    rx_ready <= 1'b0;
                    rx_sample_cnt <= 4'd0;
                    one_counts    <= 5'd0;
                    if (rx == 1'b0) begin
                        rx_state <= RX_START;
                    end
                end

                RX_START: begin
                    one_counts <= one_counts + ~rx;
                    if (rx_sample_cnt == (OVERSAMPLE - 1)) begin
                        if (one_counts >= (OVERSAMPLE/2)) begin
                            rx_state      <= RX_DATA;
                            rx_bit_cnt    <= 3'd0;
                        end else begin
                            rx_state <= RX_IDLE;
                        end
                        rx_sample_cnt <= 4'd0;
                        one_counts    <= 5'd0;
                    end else begin
                        rx_sample_cnt <= rx_sample_cnt + 1;
                    end
                end

                RX_DATA: begin
                    one_counts <= one_counts + rx;
                    if (rx_sample_cnt == (OVERSAMPLE - 1)) begin
                        rx_shift_reg <= { (one_counts >= (OVERSAMPLE/2)) ? 1'b1 : 1'b0, rx_shift_reg[7:1] };
                        rx_sample_cnt <= 4'd0;
                        one_counts    <= 5'd0;
                        if (rx_bit_cnt == 3'd7) begin
                            rx_state <= RX_STOP;
                        end else begin
                            rx_bit_cnt <= rx_bit_cnt + 1;
                        end
                    end else begin
                        rx_sample_cnt <= rx_sample_cnt + 1;
                    end
                end

                RX_STOP: begin
                    one_counts <= one_counts + rx;
                    if (rx_sample_cnt == (OVERSAMPLE - 1)) begin
                        if (one_counts >= (OVERSAMPLE/2)) begin
                            rx_Data  <= rx_shift_reg;
                            rx_ready <= 1'b1;
                        end
                        rx_state      <= RX_IDLE;
                        rx_sample_cnt <= 4'd0;
                        one_counts    <= 5'd0;
                    end else begin
                        rx_sample_cnt <= rx_sample_cnt + 1;
                    end
                end

                default: rx_state <= RX_IDLE;
            endcase
        end
    end

endmodule