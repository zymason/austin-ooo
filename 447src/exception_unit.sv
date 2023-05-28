/**
 * exception_unit.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is the exception unit, which is responsible for handling exceptions.
 *
 * The exception unit looks at all the exception signals in the processor,
 * determines the one with the highest priority, and report is to the
 * processor by setting the code. It also controls loading any other
 * registers containing metadata about the exception, such as the bad virtual
 * address (BVA) register.
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
 * Exception Unit Module
 *----------------------------------------------------------------------------*/

/**
 * The unit of the processor responsible for handling exceptions.
 *
 * This module looks at the processor's exception signals, and sets the cause
 * and other registers about the exception appropriately.
 *
 * Inputs:
 *  - rst_l             The asynchronous, active-low reset for the processor.
 *  - IBE               Instruction bus error (IBE) exception occurred.
 *  - DBE               Data bus error (DBE) exception occurred.
 *  - RI                Reserved (illegal) instruction (RI) exception occurred.
 *  - Ov                Integer overflow (Ov) exception occurred.
 *  - BP                Breakpoint (BP) exception occurred.
 *  - AdEL_instr        Address error on load (AdEL) for instruction memory
 *                      occurred.
 *  - AdEL_data         Address error on load (AdEL) for data memory occurred.
 *  - AdES              Address error on store (AdES) occurred.
 *  - CpU               Coprocessor unusable (CpU) exception occurred.
 *
 * Output:
 *  - exception_halt    Indicates the processor should halt because of an
 *                      exception.
 *  - load_excpt_regs   Indicates that the exception registers should be loaded.
 *  - load_bva_reg      Indicates that the bad virtual address (BVA) register
 *                      should be loaded.
 *  - load_bva_sel      Selects either the instruction (0) or data memory (1)
 *                      address to load into the BVA register.
 *  - cause             The code indicating the cause of the current exception.
 **/
module exception_unit
    (input  logic           rst_l, IBE, DBE, RI, Ov, BP, AdEL_instr, AdEL_data,
     input  logic           AdES, CpU,
     output logic           exception_halt, load_excpt_regs, load_bva_reg,
     output logic           load_bva_sel,
     output logic [4:0]     cause);

    // Handle when an exception occurs and loading the exception registers
    assign exception_halt = rst_l & (AdEL_instr || IBE);
    assign load_excpt_regs = rst_l & (IBE || DBE || RI || Ov || BP ||
            AdEL_instr || AdEL_data || AdES || CpU);

    // Handle when to the BVA registers and what to load into them
    assign load_bva_reg = rst_l & (AdEL_instr || AdEL_data || AdES || IBE ||
            DBE);
    assign load_bva_sel = AdEL_data || AdES || DBE;

    // Handle setting what the cause of the exception is
    always_comb begin
        if (AdEL_instr) begin
            cause = `EX_ADEL;
        end else if (AdEL_data) begin
            cause = `EX_ADEL;
        end else if (AdES) begin
            cause = `EX_ADES;
        end else if (IBE) begin
            cause = `EX_IBE;
        end else if (DBE) begin
            cause = `EX_DBE;
        end else if (CpU) begin
            cause = `EX_CPU;
        end else if (RI) begin
            cause = `EX_RI;
        end else if (Ov) begin
            cause = `EX_OV;
        end else if (BP) begin
            cause = `EX_BP;
        end else begin
            cause = `EX_INT;
        end
    end

endmodule: exception_unit

`endif /* MIPS_18447 */
