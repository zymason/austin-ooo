/**
 * syscall_unit.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is the syscall unit, which is responsible for handling system calls.
 *
 * The syscall unit looks at the value of the a0 (x10) register to determine
 * what the system call is. Naturally, the value is only considered if the
 * current instruction is a system call. The only relevant system call is
 * the exit system call, which terminates simulation. In this case, the
 * syscall unit asserts a signal indicating this.
 *
 * NOTE:
 *  - This module is deprecated, and it is intended for a MIPS processor.
 *
 * Authors:
 *  - 2004: Babak Falsafi
 *  - 2004: James Hoe
 *  - 2017: Brandon Perez
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

// This module is only included if a MIPS processor is being used
`ifdef MIPS_18447

// Local Includes
`include "exception_defines.vh"     // Exception and syscall definitions

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none

/*----------------------------------------------------------------------------
 * System Call Unit Module
 *----------------------------------------------------------------------------*/

/**
 * The unit of the processor responsible for handling syscalls.
 *
 * This module looks at the value of the a0 (x10) register to determine what
 * the syscall is. The only relevant one is the exit syscall, which terminates
 * simulation.
 *
 * Inputs:
 *  - rst_l         The asynchronous, active-low reset for the processor.
 *  - syscall       Indicates that the current instruction is a syscall.
 *  - a0_value      The value of the a0 (x10) register for the instruction.
 *
 * Output:
 *  - syscall_halt  Indicates that the processor should halt due to a syscall.
 **/
module syscall_unit
    (input  logic           rst_l, syscall,
     input  logic [31:0]    a0_value,
     output logic           syscall_halt);

     assign syscall_halt = rst_l & syscall & (a0_value == `SYS_EXIT);

endmodule: syscall_unit

`endif /* MIPS_18447 */
