/**
 * riscv_decode.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This file contains the implementation of the RISC-V decoder.
 *
 * This takes in information about the current RISC-V instruction and produces
 * the appropriate control signals to get the processor to execute the current
 * instruction.
 **/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

// RISC-V Includes
`include "riscv_isa.vh"             // RISC-V ISA definitions

// Local Includes
`include "internal_defines.vh"      // Control signals struct, ALU ops

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none

// Define a macro that prints for simulation, does nothing for synthesis
`ifdef SIMULATION_18447
`define display(print, format, arg) \
    do begin \
        if (print) begin \
            $display(format, arg); \
        end \
    end while (0)
`else
`define display(print, format, arg)
`endif /* SIMULATION_18447 */

/**
 * The instruction decoder for the RISC-V processor.
 *
 * This module processes the current instruction, determines what instruction it
 * is, and sets the control signals for the processor appropriately.
 *
 * Inputs:
 *  - rst_l             The asynchronous, active low reset for the processor.
 *  - instr             The current instruction being executed by the processor.
 *
 * Outputs:
 *  - ctrl_signals      The control signals needed to execute the given
 *                      instruction correctly.
 **/
module riscv_decode
    (input  logic           rst_l,
     input  logic [31:0]    instr,
     output ctrl_signals_t  ctrl_signals);

    // Import all of the ISA types and enums (opcodes, functions codes, etc.)
    import RISCV_ISA::*;
    
    logic rs1_n0, rs2_n0;
    assign rs1_n0 = (instr[19:15] == 5'd0) ? 1'b0 : 1'b1;
    assign rs2_n0 = (instr[24:20] == 5'd0) ? 1'b0 : 1'b1;

    // The various fields of an instruction
    opcode_t            opcode;
    funct7_t            funct7;
    rtype_funct3_t      rtype_funct3;
    itype_int_funct3_t  itype_int_funct3;
    itype_load_funct3_t itype_load_funct3;
    stype_funct3_t      stype_funct3;
    sbtype_funct3_t     sbtype_funct3;
    itype_funct12_t     itype_funct12;

    // Decode the opcode and various function codes for the instruction
    assign opcode           = opcode_t'(instr[6:0]);
    assign funct7           = funct7_t'(instr[31:25]);
    assign rtype_funct3     = rtype_funct3_t'(instr[14:12]);
    assign itype_int_funct3 = itype_int_funct3_t'(instr[14:12]);
    assign itype_load_funct3= itype_load_funct3_t'(instr[14:12]);
    assign stype_funct3     = stype_funct3_t'(instr[14:12]);
    assign sbtype_funct3    = sbtype_funct3_t'(instr[14:12]);
    assign itype_funct12    = itype_funct12_t'(instr[31:20]);

    always_comb begin
        ctrl_signals = '{
            useImm: 1'b0,
            rfWrite: 1'b0,  // never don't care
            mem2RF: 1'b0,
            pc2RF: 1'b0,
            ui2RF: 1'b0,
            aui2RF: 1'b0,
            memRead: 1'b0,  // never don't care
            memWrite: 1'b0, // never don't care
            useMul: 1'b0,
            useRS1: 1'b0,
            useRS2: 1'b0,
            imm_mode: IMM_I,
            alu_op: ALU_DC,
            ldst_mode: LDST_DC,

            btype: sbtype_funct3,
            syscall: 1'b0,
            illegal_instr: 1'b0
        };

        unique case (opcode)
            // R-type arithmetic instructions
            OP_OP: begin
                ctrl_signals.useImm = 1'b0;
                ctrl_signals.rfWrite = 1'b1;
                ctrl_signals.mem2RF = 1'b0;
                ctrl_signals.pc2RF = 1'b0;
                ctrl_signals.memRead = 1'b0;
                ctrl_signals.memWrite = 1'b0;
                ctrl_signals.useRS1 = rs1_n0;
                ctrl_signals.useRS2 = rs2_n0;
                ctrl_signals.imm_mode = IMM_I;      // Avoid don't care
                ctrl_signals.ldst_mode = LDST_DC;
                unique case (rtype_funct3)
                    // 3-bit function code for addition or subtraction
                    FUNCT3_ADD_SUB: begin
                        unique case (funct7)
                            // 7-bit function code for typical integer instructions
                            FUNCT7_INT: begin
                                ctrl_signals.alu_op = ALU_ADD;
                            end
                            // 7-bit function code for multiplication
                            FUNCT7_MULDIV: begin
                                ctrl_signals.useMul = 1'b1;
                            end
                            // 7-bit function for alternate integer instructions
                            FUNCT7_ALT_INT: begin
                                ctrl_signals.alu_op = ALU_SUB;
                            end
                            default: begin
                                `display(rst_l, "Encountered unknown/unimplemented 7-bit function code 0x%02x.",
                                        funct7);
                                ctrl_signals.illegal_instr = 1'b1;
                            end
                        endcase
                    end

                    // 3-bit function code for left-shifts
                    FUNCT3_SLL: begin
                        unique case (funct7)
                            // Only valid 7-bit function
                            FUNCT7_INT: begin
                                ctrl_signals.alu_op = ALU_SLL;
                            end
                            default: begin
                                `display(rst_l, "Encountered unknown/unimplemented 7-bit function code 0x%02x.",
                                        funct7);
                                ctrl_signals.illegal_instr = 1'b1;
                            end
                        endcase
                    end

                    // 3-bit function code for set on less than signed compare
                    FUNCT3_SLT: begin
                        unique case (funct7)
                            // Only valid 7-bit function
                            FUNCT7_INT: begin
                                ctrl_signals.alu_op = ALU_SLT;
                            end
                            default: begin
                                `display(rst_l, "Encountered unknown/unimplemented 7-bit function code 0x%02x.",
                                        funct7);
                                ctrl_signals.illegal_instr = 1'b1;
                            end
                        endcase
                    end

                    // 3-bit function code for set on less than unsigned compare
                    FUNCT3_SLTU: begin
                        unique case (funct7)
                            // Only valid 7-bit function
                            FUNCT7_INT: begin
                                ctrl_signals.alu_op = ALU_SLTU;
                            end
                            default: begin
                                `display(rst_l, "Encountered unknown/unimplemented 7-bit function code 0x%02x.",
                                        funct7);
                                ctrl_signals.illegal_instr = 1'b1;
                            end
                        endcase
                    end

                    // 3-bit function code for bitwise XOR
                    FUNCT3_XOR: begin
                        unique case (funct7)
                            // Only valid 7-bit function
                            FUNCT7_INT: begin
                                ctrl_signals.alu_op = ALU_XOR;
                            end
                            default: begin
                                `display(rst_l, "Encountered unknown/unimplemented 7-bit function code 0x%02x.",
                                        funct7);
                                ctrl_signals.illegal_instr = 1'b1;
                            end
                        endcase
                    end

                    // 3-bit function code for right-shifts
                    FUNCT3_SRL_SRA: begin
                        unique case (funct7)
                            // 7-bit function code for logical right shift
                            FUNCT7_INT: begin
                                ctrl_signals.alu_op = ALU_SRL;
                            end
                            // 7-bit function for arithmetic right shift
                            FUNCT7_ALT_INT: begin
                                ctrl_signals.alu_op = ALU_SRA;
                            end
                            default: begin
                                `display(rst_l, "Encountered unknown/unimplemented 7-bit function code 0x%02x.",
                                        funct7);
                                ctrl_signals.illegal_instr = 1'b1;
                            end
                        endcase
                    end

                    // 3-bit function code for bitwise OR
                    FUNCT3_OR: begin
                        unique case (funct7)
                            // Only valid 7-bit function
                            FUNCT7_INT: begin
                                ctrl_signals.alu_op = ALU_OR;
                            end
                            default: begin
                                `display(rst_l, "Encountered unknown/unimplemented 7-bit function code 0x%02x.",
                                        funct7);
                                ctrl_signals.illegal_instr = 1'b1;
                            end
                        endcase
                    end

                    // 3-bit function code for bitwise AND
                    FUNCT3_AND: begin
                        unique case (funct7)
                            // Only valid 7-bit function
                            FUNCT7_INT: begin
                                ctrl_signals.alu_op = ALU_AND;
                            end
                            default: begin
                                `display(rst_l, "Encountered unknown/unimplemented 7-bit function code 0x%02x.",
                                        funct7);
                                ctrl_signals.illegal_instr = 1'b1;
                            end
                        endcase
                    end

                    default: begin
                        `display(rst_l, "Encountered unknown/unimplemented 3-bit rtype integer function code 0x%01x.",
                                rtype_funct3);
                        ctrl_signals.illegal_instr = 1'b1;
                    end
                endcase
            end

            // General I-type arithmetic operation
            OP_IMM: begin
                ctrl_signals.useImm = 1'b1;
                ctrl_signals.rfWrite = 1'b1;
                ctrl_signals.mem2RF = 1'b0;
                ctrl_signals.pc2RF = 1'b0;
                ctrl_signals.memRead = 1'b0;
                ctrl_signals.memWrite = 1'b0;
                ctrl_signals.useRS1 = rs1_n0;
                ctrl_signals.imm_mode = IMM_I;
                ctrl_signals.ldst_mode = LDST_DC;

                unique case (itype_int_funct3)
                    // 3-bit function code for addition
                    FUNCT3_ADDI: begin
                        ctrl_signals.alu_op = ALU_ADD;
                    end

                    // 3-bit function code for set if less than immediate
                    FUNCT3_SLTI: begin
                        ctrl_signals.alu_op = ALU_SLT;
                    end

                    // 3-bit function code for sit if less than imm, unsigned
                    FUNCT3_SLTIU: begin
                        ctrl_signals.alu_op = ALU_SLTU;
                    end

                    // 3-bit function code for bitwise XOR
                    FUNCT3_XORI: begin
                        ctrl_signals.alu_op = ALU_XOR;
                    end

                    // 3-bit function code for bitwise OR
                    FUNCT3_ORI: begin
                        ctrl_signals.alu_op = ALU_OR;
                    end

                    // 3-bit function code for bitwise AND
                    FUNCT3_ANDI: begin
                        ctrl_signals.alu_op = ALU_AND;
                    end

                    // 3-bit function code for shift left with immediate
                    FUNCT3_SLLI: begin
                        ctrl_signals.alu_op = ALU_SLL;
                    end

                    // 3-bit function code for right-shift with immediate
                    FUNCT3_SRLI_SRAI: begin
                        case (funct7)
                            // Use logical shift
                            FUNCT7_INT: begin
                                ctrl_signals.alu_op = ALU_SRL;
                            end
                            // Use arithmetic shift (sign-extension)
                            FUNCT7_ALT_INT: begin
                                ctrl_signals.alu_op = ALU_SRA;
                            end
                            default: begin
                                `display(rst_l, "Encountered unknown/unimplemented 7-bit function code 0x%02x.",
                                        funct7);
                                ctrl_signals.illegal_instr = 1'b1;
                            end
                        endcase
                    end
                    default: begin
                        `display(rst_l, "Encountered unknown/unimplemented 3-bit itype integer function code 0x%01x.",
                                itype_int_funct3);
                        ctrl_signals.illegal_instr = 1'b1;
                    end
                endcase
            end

            // Load instructions (I-type)
            OP_LOAD: begin
                ctrl_signals.useImm = 1'b1;
                ctrl_signals.rfWrite = 1'b1;
                ctrl_signals.mem2RF = 1'b1;
                ctrl_signals.pc2RF = 1'b0;
                ctrl_signals.memRead = 1'b1;
                ctrl_signals.memWrite = 1'b0;
                ctrl_signals.useRS1 = rs1_n0;
                ctrl_signals.imm_mode = IMM_I;
                ctrl_signals.alu_op = ALU_ADD;

                unique case (itype_load_funct3)
                    // Load byte and sign-extend
                    FUNCT3_LB: begin
                        ctrl_signals.ldst_mode = LDST_B;
                    end

                    // Load half-word (2 bytes) and sign-extend
                    FUNCT3_LH: begin
                        ctrl_signals.ldst_mode = LDST_H;
                    end

                    // Load full-word (4 bytes)
                    FUNCT3_LW: begin
                        ctrl_signals.ldst_mode = LDST_W;
                    end

                    // Load byte unsigned
                    FUNCT3_LBU: begin
                        ctrl_signals.ldst_mode = LDST_BU;
                    end

                    // Load half-word unsigned
                    FUNCT3_LHU: begin
                        ctrl_signals.ldst_mode = LDST_HU;
                    end
                    default: begin
                        `display(rst_l, "Encountered unknown/unimplemented 3-bit itype load code 0x%01x.",
                                itype_load_funct3);
                        ctrl_signals.illegal_instr = 1'b1;
                        ctrl_signals.ldst_mode = LDST_DC;
                    end
                endcase
            end

            // Store instructions (S-type)
            OP_STORE: begin
                ctrl_signals.useImm = 1'b1;
                ctrl_signals.rfWrite = 1'b0;
                ctrl_signals.mem2RF = 1'b0;
                ctrl_signals.pc2RF = 1'b0;
                ctrl_signals.memRead = 1'b0;
                ctrl_signals.memWrite = 1'b1;
                ctrl_signals.useRS1 = rs1_n0;
                ctrl_signals.useRS2 = rs2_n0;
                ctrl_signals.imm_mode = IMM_S;
                ctrl_signals.alu_op = ALU_ADD;

                unique case (stype_funct3)
                    // Store byte
                    FUNCT3_SB: begin
                        ctrl_signals.ldst_mode = LDST_B;
                    end

                    // Store half-word
                    FUNCT3_SH: begin
                        ctrl_signals.ldst_mode = LDST_H;
                    end

                    // Store full word
                    FUNCT3_SW: begin
                        ctrl_signals.ldst_mode = LDST_W;
                    end
                    default: begin
                        `display(rst_l, "Encountered unknown/unimplemented 3-bit sbtype branch code 0x%01x.",
                                sbtype_funct3);
                        ctrl_signals.illegal_instr = 1'b1;
                        ctrl_signals.ldst_mode = LDST_DC;
                    end
                endcase
            end

            // LUI instruction (U-type)
            OP_LUI: begin
                ctrl_signals.useImm = 1'b1;
                ctrl_signals.rfWrite = 1'b1;
                ctrl_signals.mem2RF = 1'b0;
                ctrl_signals.pc2RF = 1'b0;
                ctrl_signals.ui2RF = 1'b1;
                ctrl_signals.memRead = 1'b0;
                ctrl_signals.memWrite = 1'b0;
                ctrl_signals.imm_mode = IMM_U;
                ctrl_signals.alu_op = ALU_ADD;
                ctrl_signals.ldst_mode = LDST_DC;
                // No funct3 is used here since we only use the immediate
            end

            // AUIPC instruction (U-type)
            OP_AUIPC: begin
                ctrl_signals.useImm = 1'b1;
                ctrl_signals.rfWrite = 1'b1;
                ctrl_signals.mem2RF = 1'b0;
                ctrl_signals.pc2RF = 1'b0;
                ctrl_signals.aui2RF = 1'b1;
                ctrl_signals.memRead = 1'b0;
                ctrl_signals.memWrite = 1'b0;
                ctrl_signals.imm_mode = IMM_U;
                ctrl_signals.alu_op = ALU_ADD;   // Addition performed in ALU
                ctrl_signals.ldst_mode = LDST_DC;
            end

            // JAL instruction (UJ-type)
            OP_JAL: begin
                ctrl_signals.useImm = 1'b1;
                ctrl_signals.rfWrite = 1'b1;
                ctrl_signals.mem2RF = 1'b0;
                ctrl_signals.pc2RF = 1'b1;
                ctrl_signals.memRead = 1'b0;
                ctrl_signals.memWrite = 1'b0;
                ctrl_signals.imm_mode = IMM_UJ;
                ctrl_signals.alu_op = ALU_ADD;   // Addition performed in ALU
                ctrl_signals.ldst_mode = LDST_DC;
            end

            // JALR instruction (I-type)
            OP_JALR: begin
                ctrl_signals.useImm = 1'b1;
                ctrl_signals.rfWrite = 1'b1;
                ctrl_signals.mem2RF = 1'b0;
                ctrl_signals.pc2RF = 1'b1;
                ctrl_signals.memRead = 1'b0;
                ctrl_signals.memWrite = 1'b0;
                ctrl_signals.useRS1 = rs1_n0;
                ctrl_signals.imm_mode = IMM_I;  // I-type immediate
                ctrl_signals.alu_op = ALU_ADD;  // ALU computes rs1+imm
                ctrl_signals.ldst_mode = LDST_DC;

                // Check for funct3 validity
                if (itype_int_funct3 != FUNCT3_ADDI) begin
                    `display(rst_l, "Encountered unknown/unimplemented 3-bit itype integer function code 0x%01x.",
                            itype_int_funct3);
                    ctrl_signals.illegal_instr = 1'b1;
                end
            end

            // Branch instructions (B-type)
            OP_BRANCH: begin
                // Control signals are identical for all branch instructions
                ctrl_signals.useImm = 1'b1;
                ctrl_signals.rfWrite = 1'b0;
                ctrl_signals.mem2RF = 1'b0;
                ctrl_signals.pc2RF = 1'b0;
                ctrl_signals.memRead = 1'b0;
                ctrl_signals.memWrite = 1'b0;
                ctrl_signals.useRS1 = rs1_n0;
                ctrl_signals.useRS2 = rs2_n0;
                ctrl_signals.imm_mode = IMM_SB;
                ctrl_signals.alu_op = ALU_ADD;
                ctrl_signals.ldst_mode = LDST_DC;

                // The branch decision is made externally
                unique case (sbtype_funct3)
                    FUNCT3_BEQ: begin end
                    FUNCT3_BNE: begin end
                    FUNCT3_BLT: begin end
                    FUNCT3_BGE: begin end
                    FUNCT3_BLTU: begin end
                    FUNCT3_BGEU: begin end
                    default: begin
                        `display(rst_l, "Encountered unknown/unimplemented 3-bit sbtype branch code 0x%01x.",
                                sbtype_funct3);
                        ctrl_signals.illegal_instr = 1'b1;
                    end
                endcase
            end

            // General system operation
            OP_SYSTEM: begin
                unique case (itype_funct12)
                    FUNCT12_ECALL: begin
                        ctrl_signals.syscall = 1'b1;
                        ctrl_signals.useRS1 = 1'b1;
                        ctrl_signals.alu_op = ALU_ADD;
                    end

                    default: begin
                        `display(rst_l, "Encountered unknown/unimplemented 12-bit itype function code 0x%03x.",
                                itype_funct12);
                        ctrl_signals.illegal_instr = 1'b1;
                    end
                endcase
            end

            default: begin
                `display(rst_l, "Encountered unknown/unimplemented opcode 0x%02x.", opcode);
                ctrl_signals.illegal_instr = 1'b1;
            end
        endcase

        // Only assert the illegal instruction exception after reset
        ctrl_signals.illegal_instr &= rst_l;
    end

endmodule: riscv_decode

