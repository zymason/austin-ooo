/**
 * sram.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is an SRAM module that can be used by the main processor.
 *
 * The SRAM is a slightly nonstandard SRAM, with synchronous writes and
 * combinational reads. It has one read port and one write port to access it.
 * The SRAM can be leveraged by the processor to create a branch target buffer
 * (BTB) for branch prediction.
 *
 * The SRAM module is synthesizable, and should be synthesized as memory by
 * most synthesis tools.
 *
 * Authors:
 *  - 2017: James Hoe
 *  - 2017: Brandon Perez
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

// RISC-V Includes
`include "riscv_uarch.vh"           // Definition of BTB default parameters

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none

/*----------------------------------------------------------------------------
 * SRAM Module
 *----------------------------------------------------------------------------*/

/**
 * A parameterized static random access memory (SRAM) used by the processor.
 *
 * This is a synchronous write, combinational (asynchronous) read SRAM. The
 * SRAM has a single read port and a single write port. Writes do not appear
 * in memory until the next cycle. The SRAM is parameterized by the size of
 * its words, the number of words, and the value it takes on reset.
 *
 * Parameters:
 *  - NUM_WORDS     The number of words present in the SRAM memory.
 *  - WORD_WIDTH    The number of bits that each word in memory has.
 *  - RESET_VAL     The value that all memory locations hold after a reset.
 *
 * Inputs:
 *  - clk           The clock to use for the SRAM.
 *  - rst_l         The asynchronous active-low reset for the SRAM.
 *  - we            Indicates that write_data data should be written to the
 *                  write_addr address in the SRAM.
 *  - read_addr     The address from which to read the value.
 *  - write_addr    The address to which to write write_data.
 *  - write_data    The data to write to the write_addr address.
 *
 * Outputs:
 *  - read_data     The data at the read_addr address in memory.
 **/
module sram
    #(parameter                         NUM_WORDS=RISCV_UArch::BTB_NUM_WORDS,
      parameter                         WORD_WIDTH=RISCV_UArch::BTB_WORD_WIDTH,
      parameter logic [WORD_WIDTH-1:0]  RESET_VAL='b0)
    (input  logic                           clk, rst_l, we,
     input  logic [$clog2(NUM_WORDS)-1:0]   read_addr, write_addr,
     input  logic [WORD_WIDTH-1:0]          write_data,
     output logic [WORD_WIDTH-1:0]          read_data);

    // The memory for the SRAM
    logic [WORD_WIDTH-1:0]  memory[NUM_WORDS-1:0];

    // Handle initialization and writing to the memory
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l) begin
            memory <= '{default: RESET_VAL};
        end else if (we) begin
            memory[write_addr] <= write_data;
        end
    end

    // Handle reading from memory
    assign read_data = memory[read_addr];

endmodule: sram
