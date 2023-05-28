/**
 * internal_defines.vh
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This contains the definitions of constants and types that are used by the
 * core of the RISC-V processor, such as control signals and ALU operations.
**/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

`ifndef INTERNAL_DEFINES_VH_
`define INTERNAL_DEFINES_VH_

// A parameter for setting NOP's on reset and pipeline flush
parameter NOP_INSTR = {3{12'd0, 5'd0, 3'd0, 5'd0, 7'b001_0011}};

// 2nd operand immediate mode
typedef enum logic [2:0] {
    IMM_I,
    IMM_S,
    IMM_SB,
    IMM_U,
    IMM_UJ,
    IMM_DC = 'bx            // Don't care value
} imm_mode_t;

// Constants that specify which operation the ALU should perform
typedef enum logic [3:0] {
    ALU_ADD,                // Addition operation
    ALU_SUB,                // Subtraction operation
    ALU_SLL,                // Left-shift operation
    ALU_SLT,                // Set if less than operation
    ALU_SLTU,               // Set if less than unsigned operation
    ALU_XOR,                // Logical XOR operation
    ALU_SRL,                // Right-shift logical operation
    ALU_SRA,                // Right-shift arithmetic operation
    ALU_OR,                 // Logical OR operation
    ALU_AND,                // Logical AND operation
    ALU_DC = 'bx            // Don't care value
} alu_op_t;

// Load/store partial word mode
typedef enum logic [2:0] {
    LDST_W,
    LDST_H,
    LDST_HU,
    LDST_B,
    LDST_BU,
    LDST_DC = 'bx            // Don't care value
} ldst_mode_t;

// Next PC source
typedef enum logic [1:0] {
    PC_plus4,               // non-control flow
    PC_cond,                // Branch
    PC_uncond,              // JAL
    PC_indirect,            // indirect jump (JALR)
    PC_DC = 'bx		    
} pc_source_t;

/* The definition of the control signal structure, which contains all
 * microarchitectural control signals for controlling the MIPS datapath. */
typedef struct packed {
    logic useImm;           // 2nd ALU input from immediate else GPR port
    logic rfWrite;          // write GPR
    logic mem2RF;           // memory load result write to GPR
    logic pc2RF;            // PC+4 write to GPR (link)
    logic ui2RF;            // Upper immediate to GPR (also regular imm to RF)
    logic aui2RF;           // Pc + Upper Immidiate to GPR
    logic memRead;          // load instruction
    logic memWrite;         // store instruction
    logic useMul;           // Select whether to load register from multiply
    logic useRS1;           // Whether RS1 is an operand or not
    logic useRS2;           // Whether RS2 is an operand or not
    imm_mode_t imm_mode;    // immediate mode applied
    alu_op_t alu_op;        // The ALU operation to perform
    ldst_mode_t ldst_mode;  // load/store partial word mode;

    logic [2:0] btype;      // branch FUNCT3
    logic syscall;          // Indicates if the current instruction is a syscall
    logic illegal_instr;    // Indicates if the current instruction is illegal
} ctrl_signals_t;

// Sets control signals for NOP whenver necessary
// Changed NOP_CTRL to be non-rfWrite
parameter NOP_CTRL = {2'b00, 9'd0, IMM_I, ALU_ADD, LDST_DC, 5'd0};

`endif /* INTERNAL_DEFINES_VH_ */
