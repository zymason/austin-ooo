/**
 * lib.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is the library of standard components used by the RISC-V processor,
 * which includes both synchronous and combinational components.
 **/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

// RISC-V Includes
`include "riscv_uarch.vh"           // Definition of BTB default parameters

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none

/*--------------------------------------------------------------------------------------------------------------------
 * Combinational Components
 *--------------------------------------------------------------------------------------------------------------------*/

/**
 * Selects on input from INPUTS inputs to output, each of WIDTH bits.
 *
 * Parameters:
 *  - INPUTS    The number of values from which the mux can select.
 *  - WIDTH     The number of bits each value contains.
 *
 * Inputs:
 *  - in        The values from which to select, packed together as a single
 *              bit-vector.
 *  - sel       The value from the inputs to output.
 *
 * Outputs:
 *  - out       The selected output from the inputs.
 **/
module mux
    #(parameter INPUTS=0, WIDTH=0)
    (input  logic [INPUTS-1:0][WIDTH-1:0]   in,
     input  logic [$clog2(INPUTS)-1:0]      sel,
     output logic [WIDTH-1:0]               out);

    assign out = in[sel];

endmodule: mux

/**
 * Adds two numbers of WIDTH bits, with a carry in bit, producing a sum and a
 * carry out bit.
 *
 * Parameters:
 *  - WIDTH     The number of bits of the numbers being summed together.
 *
 * Inputs:
 *  - cin       The carry in to the addition.
 *  - A         The first number to add.
 *  - B         The second number to add.
 *
 * Outputs:
 *  - cout      The carry out from the addition.
 *  - sum       The result of the addition.
 **/
module adder
    #(parameter WIDTH=0)
    (input  logic               cin,
     input  logic [WIDTH-1:0]   A, B,
     output logic               cout,
     output logic [WIDTH-1:0]   sum);

     assign {cout, sum} = A + B + cin;

endmodule: adder

/*--------------------------------------------------------------------------------------------------------------------
 * Synchronous Components
 *--------------------------------------------------------------------------------------------------------------------*/

/**
 * Latches and stores values of WIDTH bits and initializes to RESET_VAL.
 *
 * This register uses an asynchronous active-low reset and a synchronous
 * active-high clear. Upon clear or reset, the value of the register becomes
 * RESET_VAL.
 *
 * Parameters:
 *  - WIDTH         The number of bits that the register holds.
 *  - RESET_VAL     The value that the register holds after a reset.
 *
 * Inputs:
 *  - clk           The clock to use for the register.
 *  - rst_l         An active-low asynchronous reset.
 *  - clear         An active-high synchronous reset.
 *  - en            Indicates whether or not to load the register.
 *  - D             The input to the register.
 *
 * Outputs:
 *  - Q             The latched output from the register.
 **/
module register
   #(parameter                      WIDTH=0,
     parameter logic [WIDTH-1:0]    RESET_VAL='b0)
    (input  logic               clk, en, rst_l, clear,
     input  logic [WIDTH-1:0]   D,
     output logic [WIDTH-1:0]   Q);

     always_ff @(posedge clk, negedge rst_l) begin
         if (!rst_l)
             Q <= RESET_VAL;
         else if (clear)
             Q <= RESET_VAL;
         else if (en)
             Q <= D;
     end

endmodule:register


/**
 * A parameterized static random access memory (SRAM) used by the processor.
 *
 * This is a synchronous write, combinational (asynchronous) read SRAM. The
 * SRAM has a single read port and a single write port. Writes do not appear
 * in memory until the next cycle. The SRAM is parameterized by the size of
 * its words, the number of words, and the value it takes on reset.
 *
 * Parameters:
 *  - NUM_ENTRIES     The number of words present in the SRAM memory.
 *  - DATA_WIDTH    The number of bits that each word in memory has.
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
module sram2
    #(parameter                         NUM_ENTRIES=64,
      parameter                         DATA_WIDTH=56,
      parameter logic [DATA_WIDTH-1:0]  RESET_VAL='b0)

    (input  logic        clk, rst_l, we,
     input  logic [29:0] read_addr, write_addr,
     input  logic [29:0] write_pc,
     input  logic [1:0]  write_state,
     output logic [29:0] read_pc,
     output logic [1:0]  read_state,
     output logic        hit);

    // Tag initialization
    logic [24:0] read_tag, write_tag;
    assign read_tag = read_addr[29:5];
    assign write_tag = write_addr[29:5];

    logic [4:0] read_idx, write_idx;
    logic wr_hit1, wr_hit2;
    assign read_idx = read_addr[4:0];
    assign write_idx = write_addr[4:0];

    // The memory for the SRAM
    logic [31:0][1:0][56:0] memory;
    logic [31:0] LRU;

    always_ff @(posedge clk, negedge rst_l) begin
        $display("write_tag: %x", write_tag);
        $display("memory1: %x", memory[write_addr][0][56:32]);
        $display("memory2: %x\n", memory[write_addr][1][56:32]);
        if (!rst_l) begin
            $display("MEMORY RESET");
            memory <= 'b0;
            LRU <= 'b0;
        end 
        else if (we) begin
            if (wr_hit1) begin
                memory[write_addr][0] <= {write_tag, write_pc, write_state};
                LRU[write_addr] <= 1'b0;
            end
            else if (wr_hit2) begin
                memory[write_addr][1] <= {write_tag, write_pc, write_state};
                LRU[write_addr] <= 1'b1;
            end
            else begin
                memory[write_addr][LRU[write_addr]] <= {write_tag, write_pc, write_state};
                LRU[write_addr] <= LRU[write_addr] ^ 1'b1;
            end
        end
    end

    assign wr_hit1 = (write_tag == memory[write_addr][0][56:32]);
    assign wr_hit2 = (write_tag == memory[write_addr][1][56:32]);

    logic [1:0][29:0] pull_pc;
    logic [1:0][24:0] pull_tag;
    logic [1:0][1:0] pull_state;
    logic hit1, hit2;

    // Handle reading from memory
    assign {pull_tag[0], pull_pc[0], pull_state[0]} = memory[read_idx][0];
    assign {pull_tag[1], pull_pc[1], pull_state[1]} = memory[read_idx][1];

    assign hit1 = (read_tag == pull_tag[0]);
    assign hit2 = (read_tag == pull_tag[1]);
    assign hit = hit1 | hit2;

    assign read_pc = hit1 ? pull_pc[0] : pull_pc[1];
    assign read_state = hit1 ? pull_state[0] : pull_state[1];

endmodule: sram2