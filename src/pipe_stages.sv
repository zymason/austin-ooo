/**
 * pipe_stages.sv
 *
 * RISC-V 32-bit processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This file contains modules for each stage of a 5-stage pipeline, and the
 * submodules that are instantiated by them. Note that these modules do not
 * contain any pipeline registers, as these are handled in riscv_core.sv.
 **/

// RISC-V Includes
`include "riscv_isa.vh"             // RISC-V ISA definitions

// Local Includes
`include "internal_defines.vh"      // Control signals struct, ALU ops
import RISCV_ISA::*;

/* Use this macro to select which branch-predictor mode the design should use.
 * Per the lab handout we can support the following modes (use values listed):
 *      1: Always-not-taken
 *      2: 1-bit counter
 *      3: 2-bit hysteresis counter */
`define BP_MODE 3

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none


/**
 * A self-contained module of the pipeline's fetch stage. It contains all of the
 * branch prediction logic as well as the PC register. The branch predictors
 * take branch resolution results computed in the E-stage to update counters.
 * The predicted address and the state used for that prediction are output and
 * passed down the pipeline for branch resolution.
 *
 * Inputs:
 * - clk            Processor clock
 * - rst_l          Asynchronous register reset
 * - halted         The global processor halt signal
 * - instr_stall_DE Instruction stall signal directly from the decode stage
 * - syscall_halt_W Processor halt on a valid syscall
 * - pc_E           PC of the instruction in the E-stage
 * - brn_tgt_E      Computed branch target of instruction in E-stage
 * - brn_taken_E    Branch target address of E-stage instruction (alu_out_E)
 * - brn_state_E    The counter state used for predicting E-stage instruction
 * - btb_update_E   Whether to update the BTB or not
 * - bcond_E        Whether we actually branch on SB-type instruction or not
 * - is_branch_E    Is the E-stage instruction an SB-type branch?
 * - brn_flush_E    1: prediciton incorrect, 0: prediction correct

 * Outputs:
 * - pc_F           The PC's current value
 * - npc_plus4_F    PC+4 value
 * - pred_pc_F      The predicted next-PC
 * - brn_state_F    Counter state used for the prediction
 * - hit_F          BTB hit signal, used only by performance counters
 **/
module fetch_stage
    (input  logic           clk, rst_l, halted,
     input  logic           instr_stall_DE, instr_move_DE, syscall_halt_W,
     input  logic [31:0]    pc_E, brn_tgt_E, brn_taken_E,
     input  logic [1:0]     brn_state_D, brn_state_E,
     input  logic           btb_update_E, bcond_E, is_branch_E, brn_flush_E,
     input  logic [1:0][31:0]   instr_F,
     output logic [1:0][31:0]   pc_F, npc_plus4_F,
     output logic [31:0]    pred_pc_F,
     output logic [1:0]     brn_state_F, is_branch_F,
     output logic           hit_F);

    import RISCV_ISA::*;
    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;

    logic [31:0] next_pc;
    logic [29:0] btb_pc, actual_pred;
    logic [31:0] actual_npc, actual_pred_npc;
    logic [1:0] is_branch;
    opcode_t opcode0, opcode1;

    assign opcode0 = opcode_t'(instr_F[0][6:0]);
    assign opcode1 = opcode_t'(instr_F[1][6:0]);
    assign is_branch[0] = (opcode0 == OP_BRANCH) || (opcode0 == OP_JAL) || (opcode0 == OP_JALR);
    assign is_branch[1] = (opcode1 == OP_BRANCH) || (opcode1 == OP_JAL) || (opcode1 == OP_JALR);
    assign is_branch_F = is_branch;

    BTB2 b2_bit (.clk, .rst_l, .predict_addr(pc_F[0][31:2]), .update_addr(pc_E[31:2]),
            .branch_tgt(brn_taken_E[31:2]), .pc_plus4(pc_F[0][31:2]+30'd1),
            .prev_state(brn_state_E), .update(btb_update_E), .bcond(bcond_E),
            .is_branch(is_branch_E), .prediction(btb_pc), .actual_pred(actual_pred), .cur_state(brn_state_F), .hit(hit_F));
    assign pred_pc_F = {btb_pc, 2'b00};
    assign actual_pred_npc = {actual_pred, 2'b00};

    // If misprediction, use copmuted PC instead of predicted
    assign npc_plus4_F[0] = pc_F[0] + 32'd8;

    // Mux was bypassed by this always_comb block
    always_comb begin
        if (brn_flush_E) begin
            next_pc = brn_tgt_E;        // next_pc is architectural
            actual_npc = brn_tgt_E;     // actual_npc is for superscalar PC+8
        end
        else if (instr_move_DE) begin
            next_pc = pc_F[1];
            actual_npc = is_branch[0] ? pc_F[0] : pc_F[1];
        end
        else if (is_branch[0]) begin
            next_pc = pred_pc_F;
            // Not able to utilize prediction for two branches in a row
            actual_npc = is_branch[1] ? (brn_state_F[1] ?
                    ((actual_pred_npc == pc_F[1]) ? pc_F[0] + 32'd8 : actual_pred_npc)
                    : pc_F[1])
                    : ((actual_pred_npc == pc_F[1]) ? pc_F[0] + 32'd8 : actual_pred_npc);
        end
        else if (is_branch[1]) begin
            next_pc = pc_F[1];
            actual_npc = pc_F[1];
        end
        else begin
            next_pc = pred_pc_F;
            actual_npc = actual_pred_npc;
        end
    end

    assign pc_F[1] = pc_F[0] + 32'd4;
    assign npc_plus4_F[1] = pc_F[0] + 32'd12;

    // PC Register
    register #($bits(pc_F[0]), USER_TEXT_START) PC_Register(.clk, .rst_l,
            .en(~halted & ~instr_stall_DE & ~syscall_halt_W),
            .clear(1'b0), .D(actual_npc), .Q(pc_F[0]));

endmodule : fetch_stage



/**
 * A mostly-self contained module of the pipeline's decode stage. It contains
 * the decoder, immediate generation logic, and register file. There is
 * additional logic within the stage not in this module, including the fwd/stall
 * logic and control signal override.
 *
 * Inputs:
 * - clk            Processor clock
 * - rst_l          Asynchronous register reset
 * - halted         The global processor halt signal
 * - instr_D        Current instruction out of the F2D register bank
 * - rd_data_W      Data to be written back to the register
 * - rd_W           rd of the write-back instruction
 * - rd_we_W        Register file writes are controlled by the write-back stage

 * Outputs:
 * - rs1_data_D     Data read from rs1
 * - rs2_data_D     Data read from rs2
 * - imm_val_D      Immediate value from the current instr
 * - rd_D           Decoded rd from the current instr
 * - rs1_D          Decoded rs1 from the current instr
 * - rs2_D          Decoded rs2 from the current instr
 * - ctrl_signals_D Control signals based on the current instr
 **/
module decode_stage
    (input  logic               clk, rst_l, halted,
     input  logic [1:0][31:0]   instr_D,
     input  logic [1:0][31:0]   rd_data_W,  // Value to write from W stage
     input  logic [1:0][4:0]    rd_W,       // Destination from W stage
     input  logic [1:0]         rd_we_W,    // W stage controls write-enable

     output logic [1:0][31:0]   rs1_data_D, rs2_data_D, imm_val_D,
     output logic [1:0][4:0]    rd_D, rs1_D, rs2_D,
     output ctrl_signals_t [1:0] ctrl_signals_D);

    import RISCV_ISA::*;
    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;

    // Support parallel decode
    riscv_decode Decoder1 (.rst_l, .instr(instr_D[0]), .ctrl_signals(ctrl_signals_D[0]));
    riscv_decode Decoder2 (.rst_l, .instr(instr_D[1]), .ctrl_signals(ctrl_signals_D[1]));

    // Register file instantiation and register operand decoding
    assign  rs1_D[0]    = ctrl_signals_D[0].syscall ? 5'd10 : instr_D[0][19:15];
    assign  rs2_D[0]    = instr_D[0][24:20];
    assign  rd_D[0]     = instr_D[0][11:7];

    assign  rs1_D[1]    = ctrl_signals_D[1].syscall ? 5'd10 : instr_D[1][19:15];
    assign  rs2_D[1]    = instr_D[1][24:20];
    assign  rd_D[1]     = instr_D[1][11:7];

    register_file #(.FORWARD(1), .WAYS(2)) Reg_File (.rd_we(rd_we_W), .rs1(rs1_D), .rs2(rs2_D), .rd(rd_W),
                .rd_data(rd_data_W), .rs1_data(rs1_data_D), .rs2_data(rs2_data_D), .*);

    // Generate immediates
    // Support parallel decode
    riscv_imm_decode Imm1 (.instruction(instr_D[0]), .ctrl_signals(ctrl_signals_D[0]),
                .imm(imm_val_D[0]));
    riscv_imm_decode Imm2 (.instruction(instr_D[1]), .ctrl_signals(ctrl_signals_D[1]),
                .imm(imm_val_D[1]));

endmodule : decode_stage



/**
 * A module of the pipeline's execute stage. It contains the ALU, branch logic,
 * and instantiation of the multiplier. The multiplier output arrives as if it
 * were in the memory stage due to it's pipelined implementation.
 *
 * Inputs:
 * - clk            Processor clock
 * - imm_val_E      Immediate value from the D2E register bank
 * - rs1_data_E     Pipelined rs1 data, possibly forwarded
 * - rs2_data_E     Pipelined rs2 data, possibly forwarded
 * - pc_E           Pipelined pc value used for branch computation
 * - ctrl_signals_E Control signals of the current instr

 * Outputs:
 * - alu_out_E      ALU output
 * - mult_out_E     Multiplier output (timing occurs in memory stage)
 * - sb_bcond_E     Branch condition based on RF[rs1] - RF[rs2]
 **/
module execute_stage
    (input  logic               clk,
     input  logic [1:0][31:0]   imm_val_E,
     input  logic [1:0][31:0]   rs1_data_E, rs2_data_E, pc_E,
     input  ctrl_signals_t [1:0] ctrl_signals_E,
     output logic [1:0][31:0]   alu_out_E, mult_out_E,
     output logic               sb_bcond_E);

    import RISCV_ISA::*;
    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;

    logic [1:0][31:0] alu_src1, alu_src2;
    logic [31:0] sb_sub;
    imm_mode_t [1:0] imm_mode;

    assign imm_mode[0] = ctrl_signals_E[0].imm_mode;
    assign imm_mode[1] = ctrl_signals_E[1].imm_mode;

    // Explicit mux had slower timing
    assign alu_src2[0] = ctrl_signals_E[0].useImm ? imm_val_E[0] : rs2_data_E[0];
    assign alu_src2[1] = ctrl_signals_E[1].useImm ? imm_val_E[1] : rs2_data_E[1];

    // Determine alu_src1
    always_comb begin
        if (ctrl_signals_E[0].ui2RF)
            alu_src1[0] = 32'd0;
        else if (ctrl_signals_E[0].aui2RF | (imm_mode[0] == IMM_SB) | (imm_mode[0] == IMM_UJ))
            alu_src1[0] = pc_E[0];
        else
            alu_src1[0] = rs1_data_E[0];
    end
    always_comb begin
        if (ctrl_signals_E[1].ui2RF)
            alu_src1[1] = 32'd0;
        else if (ctrl_signals_E[1].aui2RF | (imm_mode[1] == IMM_SB) | (imm_mode[1] == IMM_UJ))
            alu_src1[1] = pc_E[1];
        else
            alu_src1[1] = rs1_data_E[1];
    end

    // PC+imm is computed in the ALU for control flow instructions
    riscv_alu ALU1 (.alu_src1(alu_src1[0]), .alu_src2(alu_src2[0]), .alu_op(ctrl_signals_E[0].alu_op),
            .alu_out(alu_out_E[0]));
    riscv_alu ALU2 (.alu_src1(alu_src1[1]), .alu_src2(alu_src2[1]), .alu_op(ctrl_signals_E[1].alu_op),
            .alu_out(alu_out_E[1]));

    // Branch condition decisions, comparison based on this independent SUB
    assign sb_sub = rs1_data_E[0] - rs2_data_E[0];
    riscv_branch_cond BR_GEN (.alu_output(sb_sub), .sign1(rs1_data_E[0][31]),
            .sign2(rs2_data_E[0][31]), .ctrl_signals(ctrl_signals_E[0]), .bcond(sb_bcond_E));

    multcsa #(1, 32) mult1 (.A(rs1_data_E[0]), .B(rs2_data_E[0]), .O(mult_out_E[0]), .CLK(clk));
    multcsa #(1, 32) mult2 (.A(rs1_data_E[1]), .B(rs2_data_E[1]), .O(mult_out_E[1]), .CLK(clk));
endmodule : execute_stage



/**
 * Computes the data to write into memory and the position it should be placed
 * based on control signals and the given address. Outputs the raw data to be stored
 * as well as the mask to control byte-wise writing.
 *
 * Inputs:
 * - alu_out_M      The computed address from the execute stage
 * - rs2_data_M     Data to be written to memory
 * - ctrl_signals_M A copy of the control signals from the decoder
 *
 * Outputs:
 * - data_store_M       The processed data to actually be written
 * - data_store_mask_M  Byte-enabled bit mask to select which bytes of data_store to write
 **/
module memory_write_stage
    (input  logic [31:0]    alu_out_M, rs2_data_M,
     input  ctrl_signals_t  ctrl_signals_M,
     output logic [31:0]    data_store_M,
     output logic [3:0]     data_store_mask_M);

    import RISCV_ISA::*;
    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;

    // Position within address
    logic [1:0] pos;
    assign pos = alu_out_M[1:0];

    // Handled addresses outside this module

    riscv_mem_write_ctrl MW1 (.data_to_write(rs2_data_M), .pos, .ctrl_signals(ctrl_signals_M),
            .data_store(data_store_M), .data_store_mask(data_store_mask_M));

endmodule : memory_write_stage



/**
 * Receives data from memory and formats it as requested by the instruction.
 * Includes all sign-extension logic for LH and LB.
 *
 * Inputs:
 * - data_load_M        The data read from memory
 * - alu_out_M          The address previously computed in the ALU
 * - ctrl_signals_M     A copy of the control signals from the decoder
 *
 * Outputs:
 * - data_recv_M    The data to be stored in the register file on a memory read
 **/
module memory_read_stage
    (input  logic [31:0]    data_load_M, alu_out_M,
     input  ctrl_signals_t  ctrl_signals_M,
     output logic [31:0]    data_recv_M);

    import RISCV_ISA::*;
    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;

    logic [1:0] pos;
    assign pos = alu_out_M[1:0];

    // Handled address truncation outside this module
    riscv_mem_read_ctrl MR1 (.data_load(data_load_M), .pos,
            .ctrl_signals(ctrl_signals_M), .data_recv(data_recv_M));

endmodule : memory_read_stage



/**
 * A module of the pipeline's write-back stage. Contains the selection logic for
 * what data should be written to the register file
 *
 * Inputs:
 * - mult_out_W     Output from the multiplier
 * - alu_out_W      ALU value computed in the E-stage
 * - data_recv_W    Processed data that was read from memory
 * - ctrl_signals_W Control signals of the current instr

 * Outputs:
 * - rd_data_W      Data to be written into the register
 * - rd_we_W        Write-enable of the register file
 **/
module write_stage
    (input  logic [1:0][31:0]   mult_out_W, alu_out_W, data_recv_W,
     input  ctrl_signals_t [1:0] ctrl_signals_W,
     output logic [1:0][31:0]   rd_data_W,
     output logic [1:0]         rd_we_W);

    import RISCV_ISA::*;
    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;

    logic [1:0][31:0] rd_data;

    // Use rd_data (output from rd module) as non-multiply path
    assign rd_data_W[0] = ctrl_signals_W[0].useMul ? mult_out_W[0] : rd_data[0];
    assign rd_data_W[1] = ctrl_signals_W[1].useMul ? mult_out_W[1] : rd_data[1];

    // Duplicated register write controllers for superscalar
    riscv_rd_data RD_G1 (.alu_out(alu_out_W[0]),
            .data_recv(data_recv_W[0]), .ctrl_signals(ctrl_signals_W[0]),
            .rd_data(rd_data[0]), .rd_we(rd_we_W[0]));

    riscv_rd_data RD_G2 (.alu_out(alu_out_W[1]),
            .data_recv(data_recv_W[1]), .ctrl_signals(ctrl_signals_W[1]),
            .rd_data(rd_data[1]), .rd_we(rd_we_W[1]));

endmodule : write_stage



/**
 * A module of the pipeline's forwarding and stalling logic. It takes in many
 * data and control signals from various stages, and deciphers them to output
 * register operands for use by the instr in the decode stage.
 *
 * Inputs:
 * - rd_*           The register destination of instrs in the E/M/W stages
 * - rs1_*          rs1 of the instr in the D/E stages (D for stall, E for fwd)
 * - rs2_*          rs2 of the instr in the D/E stages (D for stall, E for fwd)
 * - ctrl_signals_* Control signals from instrs in the D/E/M/W stages
 * - opcode_*       Opcode of the instr in D/E stages (D for stall, E for fwd)
 * - rs1_data_E     Data read from from rs1 after D2E register
 * - rs2_data_E     Data read from from rs2 after D2E register
 * - alu_out_*      ALU output from instrs in the M/W stages
 * - data_recv_W    Processed data that was read from memory
 * - mult_out_W     The multiplier output after M2W register

 * Outputs:
 * - rs1_fwd_data_DEMW  The new rs1 data going into the D2E register bank
 * - rs2_fwd_data_DEMW  The new rs2 data going into the D2E register bank
 * - instr_stall_DE     Stall signal (only occurs on memory reads and multiply)
 * - instr_move_DE      Move instruction in P2_D onto P1_D in next cycle
 * - instr2_hazard_D    Signal for performance tracking
 * - instr2_non_ALU_D   Signal for performance tracking
 **/
module forward_stall_logic
    (input  logic [1:0][4:0]        rd_D, rd_E, rd_M, rd_W,
     input  logic [1:0][4:0]        rs1_D, rs2_D, rs1_E, rs2_E,
     input  ctrl_signals_t [1:0]    ctrl_signals_D, ctrl_signals_E, ctrl_signals_M, ctrl_signals_W,
     input  opcode_t [1:0]          opcode_D, opcode_E,

     input  logic [1:0][31:0]   rs1_data_E, rs2_data_E,
     input  logic [1:0][31:0]   alu_out_M, alu_out_W, data_recv_W, mult_out_W,

     output logic [1:0][31:0]   rs1_fwd_data_DEMW, rs2_fwd_data_DEMW,
     output logic               instr_stall_DE, instr_move_DE,
     output logic               instr2_hazard_D, instr2_non_ALU_D);

    import RISCV_ISA::*;
    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;

    logic [1:0] r1e, r2e;
    logic r1a, r2a, r1b, r2b;
    logic mov1, mov2a, mov2b;

    // Stall logic within P1 (only on hazard distance of 1)
    assign r1e[0] = (rs1_D[0] == rd_E[0]) & (ctrl_signals_E[0].memRead | ctrl_signals_E[0].useMul) & ctrl_signals_D[0].useRS1;
    assign r2e[0] = (rs2_D[0] == rd_E[0]) & (ctrl_signals_E[0].memRead | ctrl_signals_E[0].useMul) & ctrl_signals_D[0].useRS1;

    // Stall logic within P2 (only on hazard distance of 1)
    assign r1e[1] = (rs1_D[1] == rd_E[1]) & (ctrl_signals_E[1].useMul) & ctrl_signals_D[1].useRS1;
    assign r2e[1] = (rs2_D[1] == rd_E[1]) & (ctrl_signals_E[1].useMul) & ctrl_signals_D[1].useRS2;

    // Stall logic from P1 (dest) to P2 (src) (on hazard distance of 1)
    assign r1a = (rs1_D[0] == rd_E[1]) & (ctrl_signals_E[1].useMul) & ctrl_signals_D[0].useRS1;
    assign r2a = (rs2_D[0] == rd_E[1]) & (ctrl_signals_E[1].useMul) & ctrl_signals_D[0].useRS2;

    // Stall logic from P2 (dest) to P1 (src) (on hazard distance of 1)
    assign r1b = (rs1_D[1] == rd_E[0]) & (ctrl_signals_E[0].memRead | ctrl_signals_E[0].useMul) & ctrl_signals_D[1].useRS1;
    assign r2b = (rs2_D[1] == rd_E[0]) & (ctrl_signals_E[0].memRead | ctrl_signals_E[0].useMul) & ctrl_signals_D[1].useRS2;

    // assign instr_stall_DE = r1e[0] | r2e[0] | r1e[1] | r2e[1] | r1a | r2a | r1b | r2b;
    assign instr_stall_DE = r1e[0] | r2e[0] | r1a | r2a;

    // Transfer instruction from P2 to P1
    // Move all MEM instructions into P1, and simultaneous-dependent instructions
    assign mov1 = ctrl_signals_D[1].memRead | ctrl_signals_D[1].syscall;    // | ctrl_signals_D[1].memWrite
    assign mov2a = (rs1_D[1] == rd_D[0]) & ctrl_signals_D[1].useRS1 & ctrl_signals_D[0].rfWrite;
    assign mov2b = (rs2_D[1] == rd_D[0]) & ctrl_signals_D[1].useRS2 & ctrl_signals_D[0].rfWrite;

    // Transfer I2 to I1 if trying to perform dual memory access
    logic mmov1, mmov2;
    assign mmov1 = ctrl_signals_D[0].memRead & ctrl_signals_D[1].memWrite;
    assign mmov2 = ctrl_signals_D[0].memWrite & ctrl_signals_D[1].memWrite;

    assign instr_move_DE = mov1 | mov2a | mov2b | r1e[1] | r2e[1] | r1b | r2b | mmov1 | mmov2;

    // Signals for simulation
    assign instr2_hazard_D = mov2a | mov2b | r1e[1] | r2e[1] | r1b | r2b;
    assign instr2_non_ALU_D = mov1;

    // Forwarding logic for P1
    // Set new, forwarded register data based on other stage's usage
    always_comb begin
        rs1_fwd_data_DEMW[0] = rs1_data_E[0];
        rs2_fwd_data_DEMW[0] = rs2_data_E[0];

        if (ctrl_signals_E[0].useRS1) begin
            // Omit memRead from P2 since all mem instructions get placed into P1
            if (rs1_E[0] == rd_M[1] && ctrl_signals_M[1].rfWrite)
                rs1_fwd_data_DEMW[0] = alu_out_M[1];
            else if (rs1_E[0] == rd_M[0] && ctrl_signals_M[0].rfWrite)
                rs1_fwd_data_DEMW[0] = alu_out_M[0];

            // Possibly reduce mult stalls by pulling directly from mult_out_E (if timing allows)
            else if (rs1_E[0] == rd_W[1] && ctrl_signals_W[1].useMul)
                rs1_fwd_data_DEMW[0] = mult_out_W[1];
            else if (rs1_E[0] == rd_W[0] && ctrl_signals_W[0].useMul)
                rs1_fwd_data_DEMW[0] = mult_out_W[0];

            // Omit memRead from P2
            else if (rs1_E[0] == rd_W[0] && ctrl_signals_W[0].memRead)
                rs1_fwd_data_DEMW[0] = data_recv_W[0];

            else if (rs1_E[0] == rd_W[1] && ctrl_signals_W[1].rfWrite)
                rs1_fwd_data_DEMW[0] = alu_out_W[1];
            else if (rs1_E[0] == rd_W[0] && ctrl_signals_W[0].rfWrite)
                rs1_fwd_data_DEMW[0] = alu_out_W[0];
        end

        if (ctrl_signals_E[0].useRS2) begin
            // Omit memRead from P2
            if (rs2_E[0] == rd_M[1] && ctrl_signals_M[1].rfWrite)
                rs2_fwd_data_DEMW[0] = alu_out_M[1];
            else if (rs2_E[0] == rd_M[0] && ctrl_signals_M[0].rfWrite)
                rs2_fwd_data_DEMW[0] = alu_out_M[0];

            // useMul implies rfWrite
            else if (rs2_E[0] == rd_W[1] && ctrl_signals_W[1].useMul)
                rs2_fwd_data_DEMW[0] = mult_out_W[1];
            else if (rs2_E[0] == rd_W[0] && ctrl_signals_W[0].useMul)
                rs2_fwd_data_DEMW[0] = mult_out_W[0];

            // memRead implies rfWrite
            else if (rs2_E[0] == rd_W[0] && ctrl_signals_W[0].memRead)
                rs2_fwd_data_DEMW[0] = data_recv_W[0];

            else if (rs2_E[0] == rd_W[1] && ctrl_signals_W[1].rfWrite)
                rs2_fwd_data_DEMW[0] = alu_out_W[1];
            else if (rs2_E[0] == rd_W[0] && ctrl_signals_W[0].rfWrite)
                rs2_fwd_data_DEMW[0] = alu_out_W[0];
        end
    end

    // Forwarding logic for P2
    // Set new, forwarded register data based on other stage's usage
    // No need to forward from P1_E to P2_E since it's handled by instr_move_DE
    always_comb begin
        rs1_fwd_data_DEMW[1] = rs1_data_E[1];
        rs2_fwd_data_DEMW[1] = rs2_data_E[1];

        if (ctrl_signals_E[1].useRS1) begin
            // Omit memRead from P2 since all mem instructions get placed into P1
            if (rs1_E[1] == rd_M[1] && ctrl_signals_M[1].rfWrite)
                rs1_fwd_data_DEMW[1] = alu_out_M[1];
            else if (rs1_E[1] == rd_M[0] && ctrl_signals_M[0].rfWrite | ctrl_signals_M[1].memWrite)
                rs1_fwd_data_DEMW[1] = alu_out_M[0];

            // Possibly reduce mult stalls by pulling directly from mult_out_E (if timing allows)
            else if (rs1_E[1] == rd_W[1] && ctrl_signals_W[1].useMul)
                rs1_fwd_data_DEMW[1] = mult_out_W[1];
            else if (rs1_E[1] == rd_W[0] && ctrl_signals_W[0].useMul)
                rs1_fwd_data_DEMW[1] = mult_out_W[0];

            // Omit memRead from P2
            else if (rs1_E[1] == rd_W[0] && ctrl_signals_W[0].memRead)
                rs1_fwd_data_DEMW[1] = data_recv_W[0];

            else if (rs1_E[1] == rd_W[1] && ctrl_signals_W[1].rfWrite)
                rs1_fwd_data_DEMW[1] = alu_out_W[1];
            else if (rs1_E[1] == rd_W[0] && ctrl_signals_W[0].rfWrite)
                rs1_fwd_data_DEMW[1] = alu_out_W[0];
        end

        if (ctrl_signals_E[1].useRS2) begin
            // Omit memRead from P2
            if (rs2_E[1] == rd_M[1] && ctrl_signals_M[1].rfWrite)
                rs2_fwd_data_DEMW[1] = alu_out_M[1];
            else if (rs2_E[1] == rd_M[0] && ctrl_signals_M[0].rfWrite)
                rs2_fwd_data_DEMW[1] = alu_out_M[0];

            // useMul implies rfWrite
            else if (rs2_E[1] == rd_W[1] && ctrl_signals_W[1].useMul)
                rs2_fwd_data_DEMW[1] = mult_out_W[1];
            else if (rs2_E[1] == rd_W[0] && ctrl_signals_W[0].useMul)
                rs2_fwd_data_DEMW[1] = mult_out_W[0];

            // memRead implies rfWrite
            else if (rs2_E[1] == rd_W[0] && ctrl_signals_W[0].memRead)
                rs2_fwd_data_DEMW[1] = data_recv_W[0];

            else if (rs2_E[1] == rd_W[1] && ctrl_signals_W[1].rfWrite)
                rs2_fwd_data_DEMW[1] = alu_out_W[1];
            else if (rs2_E[1] == rd_W[0] && ctrl_signals_W[0].rfWrite)
                rs2_fwd_data_DEMW[1] = alu_out_W[0];
        end
    end

endmodule : forward_stall_logic



/**
 * A module of the BTB for branch prediction in lab 3. Takes in truncated tag
 * and target addresses, as well as various control signals, then updates the
 * entry in the table. Outputs a predicted branch target as well as the state
 * used so we can update it later. BTB1 uses a 1-bit branch history counter
 * represented in the upper bit of a 2-bit state signal.
 *
 * Inputs:
 * - clk            Processor clock
 * - rst_l          Asynchronous system reset
 * - predict_addr   The current PC address (used to index for prediction)
 * - update_addr    The PC of E stage (used for tagging)
 * - branch_tgt     Computed branch target from the E stage
 * - pc_plus4       PC+4 of the F-stage, used for prediction forwarding
 * - prev_state     Hysteresis counter state used by the instr in E stage
 * - update         Whether the entry in BTB should actually be updated
 * - bcond          Indicates taken/not taken computed in E stage (0: NT, 1: T)
 * - is_branch      Bit indicating whether E stage instruction is control flow

 * Outputs:
 * - prediction     The predicted, truncated PC that's either BTB or PC+4
 * - cur_state      Hysteresis counter state used for the prediction
 * - hit            BTB hit signal, used only by performance counters
 **/
module BTB1 (
    input  logic        clk, rst_l,
    input  logic [29:0] predict_addr, update_addr, branch_tgt, pc_plus4,
    input  logic [1:0]  prev_state,
    input  logic        update, bcond, is_branch,
    output logic [29:0] prediction,
    output logic [1:0]  cur_state,
    output logic        hit);

    logic [61:0] write_data;
    logic [29:0] target, tag;
    logic [1:0] state;

    // BTB/SRAM storage scheme is {tag, target, state}
    always_comb begin
        if (is_branch) begin
            case (prev_state[0])
                2'd0: begin // NT
                    if (bcond) write_data = {update_addr, branch_tgt, 2'b01};
                    else write_data = {update_addr, branch_tgt, 2'b00};
                end
                2'd1: begin // T
                    if (bcond) write_data = {update_addr, branch_tgt, 2'b01};
                    else write_data = {update_addr, branch_tgt, 2'b00};
                end
            endcase
        end
        else begin
            write_data = {update_addr, branch_tgt, 2'b01};
        end
    end

    always_comb begin
        // The if statement allows forwarding for better predicitons
        // if (predict_addr == update_addr && update) begin
        //     prediction = write_data[1] ? target : pc_plus4;
        //     cur_state = state;
        // end
        if (predict_addr != tag) begin
            prediction = pc_plus4;
            cur_state = 2'b00;
        end
        else begin
            prediction = state ? target : pc_plus4;
            cur_state = state;
        end
    end

    assign hit = (predict_addr == tag) ? 1'b1 : 1'b0;

    sram #(128, 62, 0) bt1 (.we(update), .read_addr(predict_addr[6:0]),
            .read_data({tag, target, state}), .write_addr(update_addr[6:0]),
            .write_data, .*);

endmodule: BTB1



/**
 * A module of the BTB for branch prediction in lab 3. Takes in truncated tag
 * and target addresses, as well as various control signals, then updates the
 * entry in the table. Outputs a predicted branch target as well as the state
 * used so we can update it later. BTB2 uses a 2-bit hysteresis counter for
 * tracking branch history.
 *
 * Inputs:
 * - clk            Processor clock
 * - rst_l          Asynchronous system reset
 * - predict_addr   The current PC address (used to index for prediction)
 * - update_addr    The PC of E stage (used for tagging)
 * - branch_tgt     Computed branch target from the E stage
 * - pc_plus4       PC+4 of the F-stage, used for prediction forwarding
 * - prev_state     Hysteresis counter state used by the instr in E stage
 * - update         Whether the entry in BTB should actually be updated
 * - bcond          Indicates taken/not taken computed in E stage (0: NT, 1: T)
 * - is_branch      Bit indicating whether E stage instruction is control flow

 * Outputs:
 * - prediction     The architectural predicted PC (tgt or PC+4)
 * - actual_pred    The susperscalar predicted PC (tgt or PC+8)
 * - cur_state      Hysteresis counter state used for the prediction
 * - hit            BTB hit signal, used only by performance counters
 **/
module BTB2 (
    input  logic        clk, rst_l,
    input  logic [29:0] predict_addr, update_addr, branch_tgt, pc_plus4,
    input  logic [1:0]  prev_state,
    input  logic        update, bcond, is_branch,
    output logic [29:0] prediction, actual_pred,
    output logic [1:0]  cur_state,
    output logic        hit);

    logic [61:0] write_data;
    logic [29:0] target, tag;
    logic [1:0] state, new_state;

    // Using the following unconventional counter scheme for performance
    // 0 <- 0 -> 2
    // 0 <- 1 -> 3
    // 0 <- 2 -> 3
    // 2 <- 3 -> 3

    // BTB/SRAM storage scheme is {tag, target, state}
    always_comb begin
        if (is_branch) begin
            case (prev_state)
                2'd0: begin // Strong NT
                    if (bcond) write_data = {update_addr, branch_tgt, 2'd2};
                    else write_data = {update_addr, branch_tgt, 2'd0};
                end
                2'd1: begin // Weak NT
                    if (bcond) write_data = {update_addr, branch_tgt, 2'd3};
                    else write_data = {update_addr, branch_tgt, 2'd0};
                end
                2'd2: begin // Weak T
                    if (bcond) write_data = {update_addr, branch_tgt, 2'd3};
                    else write_data = {update_addr, branch_tgt, 2'd0};
                end
                2'd3: begin // Strong T
                    if (bcond) write_data = {update_addr, branch_tgt, 2'd3};
                    else write_data = {update_addr, branch_tgt, 2'd2};
                end
            endcase
        end
        else begin
            // A small trick here: we write strongly taken if not a branch instr
            // It's only loaded into BTB and forwarded if doing JAL/JALR
            write_data = {update_addr, branch_tgt, 2'd3};
        end
    end

    always_comb begin
        // The if statement allows forwarding for better predicitons
        if (predict_addr == update_addr && update) begin
            prediction = write_data[1] ? target : pc_plus4;
            actual_pred = write_data[1] ? target : predict_addr + 30'd2;
            cur_state = write_data[1:0];
        end
        else if (~hit) begin
            prediction = pc_plus4;
            actual_pred = predict_addr + 30'd2;
            cur_state = 2'd0;       // Counter initial value
        end
        else begin
            prediction = state[1] ? target : pc_plus4;
            actual_pred = state[1] ? target : predict_addr + 30'd2;   // PC+8
            cur_state = state;
        end
    end

    // Avoids issues with x on reset
    assign hit = (predict_addr == tag);

    logic [26:0] sm_target, sm_tag;
    logic [58:0] sm_write;

    // Slicing logic
    always_comb begin
        tag = {sm_tag, predict_addr[2:0]};
        sm_write = {write_data[61:35], write_data[31:0]};
        // {24, 30, 2}
        // 8-bit index: 61:40, [21:0] 54 total
        // 7-bit index: 61:39, [22:0] 55 total
        // 6-bit index: 61:38, [23:0] 56 total
        // 5 -> 37
        // 4 -> 36
        // 3 -> 35
        // etc.
    end

    sram #(8, 59, 0) bt1 (.we(update), .read_addr(predict_addr[2:0]),
            .read_data({sm_tag, target, state}), .write_addr(update_addr[2:0]),
            .write_data(sm_write), .*);

endmodule: BTB2
