/**
 * exception_defines.vh
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This file contains various definitions for exceptions and system calls
 * in the MIPS ISA, along with some other defines and messages for the
 * exceptions.
 *
 * These names are based on the names that are given in the MIPS R2000 ISA
 * document.
 *
 * NOTE:
 *  - This file is deprecated, and it is intended for a MIPS processor.
 *
 * Authors:
 *  - 2004: Babak Falsafi
 *  - 2004: James Hoe
 **/

/*----------------------------------------------------------------------------*
 *                          DO NOT MODIFY THIS FILE!                          *
 *          You should only add or change files in the src directory!         *
 *----------------------------------------------------------------------------*/

`ifndef EXCEPTION_DEFINES_VH_
`define EXCEPTION_DEFINES_VH_

////
//// USEFUL CONSTANTS
////

// Mask for the coprocessor number from the opcode
`define OP__ZMASK       6'h03

// Mask to force word-alignment of addresses
`define ADDR_ALIGN_MASK 32'hfffffffc

// Mask particular bytes
`define BYTE_0_MASK     32'h000000ff
`define BYTE_1_MASK     32'h0000ff00
`define BYTE_2_MASK     32'h00ff0000
`define BYTE_3_MASK     32'hff000000
`define BYTE_0_1_MASK   32'h0000ffff
`define BYTE_0_2_MASK   32'h00ff00ff
`define BYTE_0_3_MASK   32'hff0000ff
`define BYTE_1_2_MASK   32'h00ffff00
`define BYTE_1_3_MASK   32'hff00ff00
`define BYTE_2_3_MASK   32'hffff0000
`define BYTE_0_1_2_MASK 32'h00ffffff
`define BYTE_0_1_3_MASK 32'hff00ffff
`define BYTE_0_2_3_MASK 32'hffff00ff
`define BYTE_1_2_3_MASK 32'hffffff00

////
//// Exception Codes
////

// Left shift amount to align exception code from cause register
`define EX__SHIFT 'd2
// Mask to extract exception code from CAUSE register
`define EX__MASK  32'h0000007c

// Exception codes
`define EX_INT    5'd0
`define EX_MOD    5'd1
`define EX_TLBL   5'd2
`define EX_TLBS   5'd3
`define EX_ADEL   5'd4
`define EX_ADES   5'd5
`define EX_IBE    5'd6
`define EX_DBE    5'd7
`define EX_SYS    5'd8
`define EX_BP     5'd9
`define EX_RI     5'd10
`define EX_CPU    5'd11
`define EX_OV     5'd12
`define EX_TR     5'd13
`define EX_VCEI   5'd14
`define EX_FPE    5'd15
`define EX_WATCH  5'd23
`define EX_VCED   5'd31

////
//// RISC-V 447 constants
////

// System calls
`define SYS_EXIT         32'h0a

// Messages
`define MSG_UNCLASS     'h0
`define MSG_UNCLASS_S   "[0x%h]"
`define MSG_EOP         'h1
`define MSG_EOP_S       "[End of program at 0x%h]"
`define MSG_SWDUMP      'h3
`define MSG_SWDUMP_S    "[Program register dump at 0x%h]"

`define MSG_EXCPT       'h10
`define MSG_EXCPT_S     "[Exception at %h]"
`define MSG_ADEL        'h11
`define MSG_ADEL_S      "[Address error exception on load at 0x%h]"
`define MSG_ADES        'h12
`define MSG_ADES_S      "[Address error exception on store at 0x%h]"
`define MSG_IBE         'h13
`define MSG_IBE_S       "[Instruction bus error exception at 0x%h]"
`define MSG_DBE         'h14
`define MSG_DBE_S       "[Data bus error exception at 0x%h]"
`define MSG_BP          'h15
`define MSG_BP_S        "[Breakpoint exception at 0x%h]"
`define MSG_RI          'h16
`define MSG_RI_S        "[Reserved instruction exception at 0x%h]"
`define MSG_CPU         'h17
`define MSG_CPU_S       "[Coprocessor unusable exception at 0x%h]"
`define MSG_OV          'h18
`define MSG_OV_S        "[Arithmetic overflow exception at 0x%h]"

`endif /* EXCEPTION_DEFINES_VH_ */
