import cocotb
from cocotb.triggers import Timer
from cocotb.clock import Clock
from cocotbext.uart import UartSource, UartSink
from cocotb.triggers import RisingEdge, ClockCycles, with_timeout
from cocotb.utils import get_sim_time

@cocotb.test()
async def test_uart_bootloader(dut):
    """Test UART bootloader handshake: command 37, expect ACK or NACK."""
    
    # Start clock
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    
    # Setup UART
    uart_source = UartSource(dut.rx, baud=115200)
    uart_sink   = UartSink(dut.tx, baud=115200)
    
    # Reset DUT
    dut._log.info("Resetting DUT...")
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 100)
    
    dut._log.info(f"Time after reset: {get_sim_time('ns')} ns")
    dut._log.info("Sending handshake command 0x25")
    
    # Send handshake command 37 (0x25)
    await uart_source.write([0x25])
    
    dut._log.info(f"Command sent at: {get_sim_time('ns')} ns")
    
    # Read response
    resp = await uart_sink.read(count=1)
    dut._log.info(f"Response received at: {get_sim_time('ns')} ns")
    dut._log.info(f"Response value: 0x{resp[0]:02X}")
    await ClockCycles(dut.clk, 5000)
    # Check response
    if resp[0] == 0x55:
        dut._log.info("✓ SUCCESS: Handshake ACK received")
        
        # Send instructions
        instructions = [0x00000013, #first instruction 
                        0x019886b3, #add x13, x17, x25 --R-type
                        0x403402b3, #sub x5, x8, x3 --R-type
                        0x3e820293, #addi x5, x4, 1000 --I-type
                        0x00512023, #sw x5, 0(x2) --S-type
                        0x00012183, #lw x3, 0(x2) --I-type
                        0x06208263, #beq x1, x2, 100 --B-type 
                        0x00000000  #last instruction    
        ]
        
        for inst in instructions:
            bytes_to_send = [
                (inst >> 0) & 0xFF,   # LSB first
                (inst >> 8) & 0xFF,
                (inst >> 16) & 0xFF,
                (inst >> 24) & 0xFF    # MSB last
            ]
            await uart_source.write(bytes_to_send)
            dut._log.info(f"Sent instruction 0x{inst:08X}")  # ← NOW INSIDE the loop!
            
            # Optional: wait a bit between instructions
            await ClockCycles(dut.clk, 20000)
        
        # Wait for memory writes to complete
        await ClockCycles(dut.clk, 10000)
        
        # Verify memory contents if you have access to memory
        dut._log.info("All instructions sent successfully")
        
    elif resp[0] == 0xFF:
        dut._log.warning(" NACK received - bootloader rejected command")
    else:
        assert False, f"Unexpected response: 0x{resp[0]:02X}"