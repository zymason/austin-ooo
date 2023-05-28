/**
 * riscv_core.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is the core part of the processor, and is responsible for executing the
 * instructions and updating the CPU state appropriately.
 *
 * This is where you can start to add code and make modifications to fully
 * implement the processor. You can add any additional files or change and
 * delete files as you need to implement the processor, provided that they are
 * under the src directory. You may not change any files outside the src
 * directory. The only requirement is that there is a riscv_core module with the
 * interface defined below, with the same port names as below.
 *
 * The Makefile will automatically find any files you add, provided they are
 * under the src directory and have either a *.v, *.vh, or *.sv extension. The
 * files may be nested in subdirectories under the src directory as well.
 * Additionally, the build system sets up the include paths so that you can
 * place header files (*.vh) in any subdirectory in the src directory, and
 * include them from anywhere else inside the src directory.
 *
 * The compiler and synthesis tools support both Verilog and System Verilog
 * constructs and syntax, so you can write either Verilog or System Verilog
 * code, or mix both as you please.
 **/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

// RISC-V Includes
`include "riscv_abi.vh"             // ABI registers and definitions
`include "riscv_isa.vh"             // RISC-V ISA definitions
`include "memory_segments.vh"       // Memory segment starting addresses

// Local Includes
`include "internal_defines.vh"      // Control signals struct, ALU ops

/* A quick switch to enable/disable tracing. Comment out to disable. Please
 * comment this out before submitting your code. You'll also want to comment
 * this out for longer tests, as it will make them run much faster. */
`define TRACE

// Force the compiler to throw an error if any variables are undeclared
`default_nettype none


/**
 * The core of the RISC-V processor, everything except main memory.
 *
 * This is the RISC-V processor, which, each cycle, fetches the next
 * instruction, executes it, and then updates the register file, memory,
 * and register file appropriately.
 *
 * The memory that the processor interacts with is dual-ported with a
 * single-cycle synchronous write and combinational read. One port is used to
 * fetch instructions, while the other is for loading and storing data.
 *
 * Inputs:
 *  - clk               The global clock for the processor.
 *  - rst_l             The asynchronous, active low reset for the processor.
 *  - instr_mem_excpt   Indicates that an invalid instruction address was given
 *                      to memory.
 *  - data_mem_excpt    Indicates that an invalid address was given to the data
 *                      memory during a load and/or store operation.
 *  - instr             The instruction loaded from the instr_addr
 *                      address in memory.
 *  - data_load         The data loaded from the data_addr address in memory.
 *
 * Outputs:
 *  - data_load_en      Indicates that data from the data_addr address in
 *                      memory should be loaded.
 *  - halted            Indicates that the processor has stopped because of a
 *                      syscall or exception. Used to indicate to the testbench
 *                      to end simulation. Must be held until next clock cycle.
 *  - data_store_mask   Byte-enable bit mask  signal indicating which bytes of data_store
 *                      should be written to the data_addr address in memory.
 *  - instr_addr        The address of the instruction to load from memory.
 *  - instr_stall       stall instruction load from memory if multicycle.
 *  - data_addr         The address of the data to load or store from memory.
 *  - data_stall        stall data load from memory if multicycle.
 *  - data_store        The data to store to the data_addr address in memory.
 **/
module riscv_core
    (input  logic           clk, rst_l, instr_mem_excpt, data_mem_excpt,
     input  logic [2:0][31:0]   instr, data_load,
     output logic           data_load_en, halted,
     output logic [3:0]     data_store_mask,
     output logic [29:0]    instr_addr, data_addr,
     output logic           instr_stall, data_stall,
     output logic [31:0]    data_store);

    /* Import the ISA field types, and the argument to ecall to halt the
     * simulator, and the start of the user text segment. */
    import RISCV_ISA::*;
    import RISCV_ABI::ECALL_ARG_HALT;
    import MemorySegments::USER_TEXT_START;


    /****  Global signals ****/
    logic exception_halt_MW, syscall_halt;


    /**** F stage signals ****/
    logic [1:0][31:0] instr_F, instr_new_F, instr_temp_F;
    logic [1:0][31:0] pc_F, pc_new_F, npc_plus4_F, npc_plus4_new_F;
    logic [31:0] pred_pc_F;
    logic [1:0]  brn_state_F;
    logic hit_F;    // Signal for performance counters
    logic clear_I2_F;


    /**** D stage signals ****/
    logic [1:0][31:0] instr_D;
    logic [1:0][31:0] pc_D, npc_plus4_D;
    logic [31:0] pred_pc_D;

    logic [1:0][31:0] rs1_data_D, rs2_data_D, imm_val_D;
    logic [1:0][31:0] rs1_fwd_data_DEMW, rs2_fwd_data_DEMW;
    logic [1:0][4:0] rd_D, rs1_D, rs2_D;
    logic [1:0][4:0] rd_new_D;
    logic [1:0] brn_state_D;
    logic [1:0] is_branch_F;
    logic instr_stall_DE, instr_move_DE, instr2_hazard_D, instr2_non_ALU_D;
    ctrl_signals_t [1:0] ctrl_signals_D, ctrl_signals_new_D;
    opcode_t [1:0] opcode_D, opcode_new_D;


    /**** E stage signals ****/
    logic [1:0][31:0] imm_val_E, rs1_data_E, rs2_data_E;
    logic [1:0][31:0] pc_E, npc_plus4_E;
    logic [31:0] pred_pc_E;
    logic [1:0][31:0] data_store_E;
    logic [1:0][3:0] data_store_mask_E;
    logic [31:0] data_store_final_E;
    logic [3:0] data_store_mask_final_E;
    logic [1:0][4:0] rd_E, rs1_E, rs2_E;
    ctrl_signals_t [1:0] ctrl_signals_E;
    opcode_t [1:0] opcode_E;

    logic [1:0][31:0] alu_out_E, mult_out_E, rd_data_E;
    logic [31:0] brn_tgt_E, brn_taken_E;
    logic [1:0] brn_state_E;
    logic btb_update_E, bcond_E, is_branch_E, brn_flush_E;
    logic [1:0] is_jump_E;
    logic sb_bcond_E;


    /**** M stage signals ****/
    logic [1:0][31:0] data_load_M, alu_out_M, rs2_data_M;
    logic [1:0][4:0] rd_M;
    ctrl_signals_t [1:0] ctrl_signals_M;

    logic [1:0][31:0] data_recv_M;
    logic [31:0] data_store_M;
    logic [29:0] data_addr_M;
    logic [3:0] data_store_mask_M;
    logic data_stall_M;
    logic mem_stall_W;


    /**** W stage signals ****/
    logic [1:0][31:0] mult_out_W, alu_out_W, data_recv_W;
    ctrl_signals_t [1:0] ctrl_signals_W;

    logic [1:0][31:0] rd_data_W;
    logic [1:0][4:0] rd_W;
    logic [1:0] rd_we_W;
    logic syscall_halt_W;


    /******** BEGIN F Stage *********/
    fetch_stage S1 (.clk, .rst_l, .halted, .instr_stall_DE, .instr_move_DE, .syscall_halt_W,
            .pc_E(pc_E[0]), .brn_tgt_E, .brn_taken_E, .brn_state_D, .brn_state_E, .btb_update_E, .bcond_E, .is_branch_E,
            .brn_flush_E, .pc_F, .npc_plus4_F, .pred_pc_F, .brn_state_F, .hit_F, .instr_F,
            .is_branch_F);

    // Interface to external I-mem
    assign instr_addr = pc_F[0][31:2];
    assign instr_F = instr[1:0];
    
    // instr_move_DE has priority over rearrangements due to branches
    // 3x 2:1 muxes setup like a 4:1
    mux #(2,32) inF2A (.in({NOP_INSTR[31:0], instr_F[0]}),
            .sel(is_branch_F[0]), .out(instr_temp_F[1]));
    mux #(2,32) inF2B (.in({NOP_INSTR[31:0], instr_F[1]}),
            .sel(is_branch_F[1]), .out(instr_temp_F[0]));
    mux #(2,32) inF2C (.in(instr_temp_F), .sel(instr_move_DE & ~brn_state_D[1]),
            .out(instr_new_F[1]));

    mux #(2,32) inF1 (.in({instr_D[1], instr_F[0]}), .sel(instr_move_DE & ~brn_state_D[1]),
            .out(instr_new_F[0]));

    always_comb begin
        // Move current P1 pc into P2.
        pc_new_F[0] = instr_move_DE & ~brn_state_D[1] ? pc_D[1] : pc_F[0];
        pc_new_F[1] = instr_move_DE & ~brn_state_D[1] ? pc_F[0] : pc_F[1];

        npc_plus4_new_F[0] = instr_move_DE & ~brn_state_D[1] ? npc_plus4_D[1] : npc_plus4_F[0];
        npc_plus4_new_F[1] = instr_move_DE & ~brn_state_D[1] ? npc_plus4_F[0] : npc_plus4_F[1];
    end
    /********  END F Stage  ********/


    /******** BEGIN F2D Stage *********/
    // Stop enable on these registers when stalling in addition to halts
    // Reset these registers to a NOP when performing a pipeline flush
    register #($bits(pred_pc_F), 0) ppc_F2D_Register(.clk, .rst_l,
            .en(~halted & ~instr_stall_DE & ~syscall_halt_W), .clear(brn_flush_E), 
            .D(pred_pc_F), .Q(pred_pc_D));
    register #($bits(brn_state_F), 0) brn_F2D_Register(.clk, .rst_l,
            .en(~halted & ~instr_stall_DE & ~syscall_halt_W), .clear(brn_flush_E),
            .D(brn_state_F), .Q(brn_state_D));
    register #($bits(instr_F), NOP_INSTR) instr_F2D_Register(.clk, .rst_l,
            .en(~halted & ~instr_stall_DE & ~syscall_halt_W), .clear(brn_flush_E),
            .D(instr_new_F), .Q(instr_D));
    register #(2*$bits(pc_D), 0) pc_F2D_Register(.clk, .rst_l,
            .en(~halted & ~instr_stall_DE & ~syscall_halt_W), .clear(brn_flush_E), 
            .D({pc_new_F, npc_plus4_new_F}), .Q({pc_D, npc_plus4_D}));

    /********  END F2D Stage  *********/
    

    /******** BEGIN D Stage ********/

    decode_stage S2 (.clk, .rst_l, .halted, .instr_D, .rd_data_W, .rd_W, .rd_we_W,
            .rs1_data_D, .rs2_data_D, .imm_val_D, .rd_D, .rs1_D, .rs2_D,
            .ctrl_signals_D);
    
    assign opcode_D[0]      = opcode_t'(instr_D[0][6:0]);
    assign opcode_D[1]      = opcode_t'(instr_D[1][6:0]);
    assign instr_stall      = instr_stall_DE;
    assign halted           = rst_l & (syscall_halt | exception_halt_MW);

    always_comb begin
        ctrl_signals_new_D = ctrl_signals_D;
        rd_new_D = rd_D;
        opcode_new_D = opcode_D;
        if (instr_move_DE) begin
            ctrl_signals_new_D[1] = NOP_CTRL;
            rd_new_D[1] = 'd0;
            opcode_new_D[1] = OP_IMM;
        end
    end

    /********  END D Stage  ********/


    /******** BEGIN D2E Stage *********/
    // Reset these registers to a NOP when performing a pipeline flush
    register #($bits(imm_val_D),0) imm_D2E_Register(.clk, .rst_l,
            .en(~halted), .clear(brn_flush_E | instr_stall_DE), .D(imm_val_D), .Q(imm_val_E));
    register #(2*$bits(pc_D),0) pc_D2E_Register(.clk, .rst_l,
            .en(~halted), .clear(brn_flush_E), .D({pc_D, npc_plus4_D}), 
            .Q({pc_E, npc_plus4_E}));
    register #($bits(pred_pc_D),0) ppc_D2E_Register(.clk, .rst_l,
            .en(~halted), .clear(brn_flush_E), .D(pred_pc_D), 
            .Q(pred_pc_E));
    register #(2*$bits(rs1_data_D),0) rs_D2E_Register(.clk, .rst_l,
            .en(~halted), .clear(brn_flush_E), .D({rs1_data_D, rs2_data_D}),
            .Q({rs1_data_E, rs2_data_E}));
    register #(3*$bits(rs1_D),0) reg_D2E_Register(.clk, .rst_l,
            .en(~halted), .clear(brn_flush_E | instr_stall_DE), .D({rd_new_D, rs1_D, rs2_D}),
            .Q({rd_E, rs1_E, rs2_E}));
    opcd_reg #(2) op_D2E_Register(.clk, .rst_l, .en(~halted), 
            .clear(brn_flush_E | instr_stall_DE), .D(opcode_new_D), .Q(opcode_E));
    register #($bits(brn_state_D),0) brn_D2E_Register(.clk, .rst_l,
            .en(~halted), .clear(brn_flush_E), .D(brn_state_D), .Q(brn_state_E));
    ctrl_reg #(2) ctrl_D2E_Register(.clk, .rst_l,
            .en(~halted), .clear(brn_flush_E | instr_stall_DE),
            .D(ctrl_signals_new_D), .Q(ctrl_signals_E));

    /********  END D2E Stage  *********/


    /******** BEGIN E Stage ********/
    execute_stage S3 (.clk, .imm_val_E, .rs1_data_E(rs1_fwd_data_DEMW), .rs2_data_E(rs2_fwd_data_DEMW),
            .ctrl_signals_E, .alu_out_E, .mult_out_E, .sb_bcond_E, .pc_E);

    // Prevents instr_moves from occurring on branch predicted taken
    logic temp_move_DE;

    forward_stall_logic SL1 (.rd_D, .rd_E, .rd_M, .rd_W, .rs1_D,
            .rs2_D, .rs1_E, .rs2_E, .ctrl_signals_D, .ctrl_signals_E,
            .ctrl_signals_M, .ctrl_signals_W,
            .opcode_D, .opcode_E, .rs1_data_E,
            .rs2_data_E, .alu_out_M, .alu_out_W,
            .data_recv_W, .mult_out_W, .rs1_fwd_data_DEMW,
            .rs2_fwd_data_DEMW, .instr_stall_DE, .instr_move_DE(temp_move_DE),
            .instr2_hazard_D, .instr2_non_ALU_D);
    
    // If predicted taken, don't perform an instruction move
    assign instr_move_DE = ~brn_state_D[1] & temp_move_DE;

    logic [1:0] flush_E;
    logic flush_temp_E;
    assign flush_E[1] = (pred_pc_E == alu_out_E[0]);
    assign flush_E[0] = (pred_pc_E == pc_E[1]);

    // Branch resolution logic
    mux #(2,32) tgt_mux (.in({alu_out_E[0], pc_E[1]}), .sel(ctrl_signals_E[0].pc2RF | sb_bcond_E), .out(brn_tgt_E));
    assign brn_taken_E = alu_out_E[0];
    mux  #(2,1) flush_mux (.in(flush_E), .sel(ctrl_signals_E[0].pc2RF | sb_bcond_E), .out(flush_temp_E));
    assign brn_flush_E = ~flush_temp_E & btb_update_E;
    assign is_jump_E[0] = ({opcode_E[0][6:4], opcode_E[0][2:0]} == 6'b110_111);
    assign is_jump_E[1] = ({opcode_E[1][6:4], opcode_E[1][2:0]} == 6'b110_111);
    assign btb_update_E = (opcode_E[0] == OP_BRANCH) | ctrl_signals_E[0].pc2RF;
    assign is_branch_E = (opcode_E[0] == OP_BRANCH);

    // Branch taken if sb_bcond_E or we have JAL/JALR
    assign bcond_E = (ctrl_signals_E[0].pc2RF | (sb_bcond_E & is_branch_E)) & (brn_tgt_E != (pc_E[0] + 32'd4));

    logic brn_imm_E;
    assign brn_imm_E = (ctrl_signals_E[0].pc2RF | (sb_bcond_E & is_branch_E)) & (brn_tgt_E == pc_E[0] + 32'd4);

    // Select appropriate source for writing to the register file
    assign rd_data_E[0] = ctrl_signals_E[0].pc2RF ? pc_E[0]+32'd4 : alu_out_E[0];
    assign rd_data_E[1] = ctrl_signals_E[1].pc2RF ? pc_E[1]+32'd4 : alu_out_E[1];

    assign data_store_final_E = ctrl_signals_E[0].memWrite ? data_store_E[0] : data_store_E[1];
    assign data_store_mask_final_E = ctrl_signals_E[0].memWrite ? data_store_mask_E[0] : data_store_mask_E[1];

    memory_write_stage SW1 (.alu_out_M(rd_data_E[0]), .rs2_data_M(rs2_fwd_data_DEMW[0]),
            .ctrl_signals_M(ctrl_signals_E[0]), .data_store_M(data_store_E[0]),
            .data_store_mask_M(data_store_mask_E[0]));

    memory_write_stage SW2 (.alu_out_M(rd_data_E[1]), .rs2_data_M(rs2_fwd_data_DEMW[1]),
            .ctrl_signals_M(ctrl_signals_E[1]), .data_store_M(data_store_E[1]),
            .data_store_mask_M(data_store_mask_E[1]));

    /********  END E Stage  ********/


    /******** BEGIN E2M Stage *********/
    // For memory multiplexing access
    register #($bits(data_store_final_E), 0) stor_E2M_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(data_store_final_E), .Q(data_store_M));
    register #($bits(data_store_mask_final_E), 0) mask_E2M_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(data_store_mask_final_E), .Q(data_store_mask_M));

    register #($bits(rs2_data_E[0]), 0) rs_E2M_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(rs2_fwd_data_DEMW[0]), .Q(rs2_data_M[0]));
    ctrl_reg #(1) ctrl_E2M_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(ctrl_signals_E[0]), .Q(ctrl_signals_M[0]));
    register #($bits(rd_E[0]), 0) reg_E2M_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(rd_E[0]), .Q(rd_M[0]));
    register #($bits(rd_data_E[0]), 0) alu_E2M_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(rd_data_E[0]), .Q(alu_out_M[0]));
    
    register #($bits(rs2_data_E[1]), 0) rs_E2M_Register1(.clk, .rst_l,
            .en(~halted), .clear(brn_flush_E), .D(rs2_fwd_data_DEMW[1]), .Q(rs2_data_M[1]));
    ctrl_reg #(1) ctrl_E2M_Register1(.clk, .rst_l,
            .en(~halted), .clear(brn_flush_E | bcond_E), .D(ctrl_signals_E[1]), .Q(ctrl_signals_M[1]));
    register #($bits(rd_E[1]), 0) reg_E2M_Register1(.clk, .rst_l,
            .en(~halted), .clear(brn_flush_E), .D(rd_E[1]), .Q(rd_M[1]));
    register #($bits(rd_data_E[1]), 0) alu_E2M_Register1(.clk, .rst_l,
            .en(~halted), .clear(brn_flush_E), .D(rd_data_E[1]), .Q(alu_out_M[1]));
            
    /********  END E2M Stage  *********/


    /******** BEGIN M Stage ********/

    memory_read_stage S4R (.data_load_M(data_load_M[0]), .alu_out_M(alu_out_M[0]),
            .ctrl_signals_M(ctrl_signals_M[0]), .data_recv_M(data_recv_M[0]));
    
    // Select which pipeline has the write operation
    mux #(2,30) addrM (.in({alu_out_M[1][31:2], alu_out_M[0][31:2]}), .sel(ctrl_signals_M[1].memWrite),
            .out(data_addr_M));

    assign data_recv_M[1] = 32'd0;
    assign data_load_M[0] = data_load[0];
    assign data_store = data_store_M;
    assign data_addr = data_addr_M;
    assign data_store_mask = ctrl_signals_M[0].memWrite | ctrl_signals_M[1].memWrite ? data_store_mask_M : 4'd0;
    assign data_load_en = ctrl_signals_M[0].memRead;
    assign data_stall = 1'b0;

    /********  END M Stage  ********/


    /******** BEGIN M2W Stage *********/
    register #($bits(rd_M), 0) reg_M2W_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(rd_M), .Q(rd_W));
    register #($bits(alu_out_M), 0) alu_M2W_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(alu_out_M), .Q(alu_out_W));
    register #($bits(data_recv_M), 0) mem_M2W_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(data_recv_M), .Q(data_recv_W));
    register #($bits(mult_out_W), 0) mul_M2W_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(mult_out_E), .Q(mult_out_W));
    ctrl_reg #(2) ctrl_M2W_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(ctrl_signals_M), .Q(ctrl_signals_W));
    
    /********  END M2W Stage  *********/

    /******** BEGIN W Stage ********/
    write_stage S5 (.mult_out_W, .alu_out_W, .data_recv_W, .ctrl_signals_W,
            .rd_data_W, .rd_we_W);

    assign syscall_halt_W = (ctrl_signals_W[0].syscall && (alu_out_W[0] == ECALL_ARG_HALT)) |
                               (ctrl_signals_W[1].syscall && (alu_out_W[1] == ECALL_ARG_HALT));
    assign syscall_halt      = syscall_halt_W;
    assign exception_halt_MW = instr_mem_excpt | data_mem_excpt | 
                            (ctrl_signals_M[0].illegal_instr & ~syscall_halt &
                            ctrl_signals_M[1].illegal_instr & ~syscall_halt);

    /********  END W Stage  ********/


`ifdef SIMULATION_18447
    always_ff @(posedge clk) begin
        if (rst_l && instr_mem_excpt) begin
            $display("Instruction memory exception at address 0x%08x.", instr_addr << 2);
        end
        if (rst_l && data_mem_excpt) begin
            $display("Data memory exception at address 0x%08x.", data_addr << 2);
        end
        if (rst_l && syscall_halt) begin
            $display("ECALL invoked with halt argument. Terminating simulation at 0x%08x.", pc_F);
        end
    end

    /****** Performance Recording ******/

    // Counter support signals
    logic hit_D, hit_E, hit_M, hit_W;
    logic invalid_F, invalid_D, invalid_E, invalid_M, invalid_W, invalid_ID, invalid_ID2;
    logic invalid_F2, invalid_D2, invalid_E2, invalid_M2, invalid_W2;
    logic brn_flush_M, brn_flush_W;
    logic instr2_hazard_FD, instr2_non_ALU_FD, instr2_hazard_E, instr2_non_ALU_E, instr2_hazard_M, instr2_non_ALU_M, instr2_hazard_W, instr2_non_ALU_W;
    logic instr2_single_D, instr2_single_E, instr2_single_M, instr2_single_W;
    logic instr_stall_E, instr_stall_M, instr_stall_W;
    logic flushed_D, flushed_E, flushed_M, flushed_W;
    logic [1:0][31:0] imm_val_M, imm_val_W;
    logic [1:0] brn_state_M, brn_state_W;
    logic [1:0][4:0] rs1_M, rs1_W;
    logic [1:0][31:0] pc_M, pc_W;
    opcode_t [1:0] opcode_M, opcode_W;
   
   logic [1:0] is_BRANCH_W, is_JAL_W, is_JALR_W, is_OP_W, is_IMM_W, is_LUI_W, is_AUIPC_W, is_LOAD_W, is_STORE_W, is_SYSTEM_W;
   
    // Signal Specifying whether the current instruction was killed
    assign invalid_ID = brn_flush_E | invalid_D | instr_stall_DE;
    register #(1,1) invalid_F2D_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0),
            .D(brn_flush_E), .Q(invalid_D));  // instr_stall_DE may be needed here
    register #(1,1) invalid_D2E_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0),
            .D(brn_flush_E | invalid_D | instr_stall_DE), .Q(invalid_E));
    register #(1,1) invalid_E2M_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0),
            .D(invalid_E), .Q(invalid_M));
    register #(1,1) invalid_M2W_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0),
            .D(invalid_M), .Q(invalid_W));
    
    assign invalid_ID2 = brn_flush_E | invalid_D2 | instr_stall_DE | instr_move_DE;
    register #(1,1) invalid_F2D_Register2 (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0),
            .D(brn_flush_E | (instr_move_DE & ~brn_state_D[1] & is_branch_F[0]) | (~(instr_move_DE & ~brn_state_D[1]) & is_branch_F[1])), .Q(invalid_D2));
    register #(1,1) invalid_D2E_Register2 (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0),
            .D(brn_flush_E | invalid_D2 | instr_stall_DE | instr_move_DE), .Q(invalid_E2));
    register #(1,1) invalid_E2M_Register2 (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0),
            .D(invalid_E2 | brn_flush_E | bcond_E), .Q(invalid_M2));
    register #(1,1) invalid_M2W_Register2 (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0),
            .D(invalid_M2), .Q(invalid_W2));
            
    // 1 Instruction Reason
    register #(1,0) hazard_D2E_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(brn_flush_E), 
            .D(instr2_hazard_D), .Q(instr2_hazard_E));
    register #(1,0) hazard_E2M_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(instr2_hazard_E), .Q(instr2_hazard_M));
    register #(1,0) hazard_M2W_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(instr2_hazard_M), .Q(instr2_hazard_W));
    
    register #(1,0) non_ALU_F2D_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(brn_flush_E), 
            .D((instr_move_DE & ~brn_state_D[1] & is_branch_F[0]) | (~(instr_move_DE & ~brn_state_D[1]) & is_branch_F[1])), .Q(instr2_non_ALU_FD));
    register #(1,0) non_ALU_D2E_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(brn_flush_E), 
            .D(instr2_non_ALU_D | instr2_non_ALU_FD), .Q(instr2_non_ALU_E));
    register #(1,0) non_ALU_E2M_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(instr2_non_ALU_E), .Q(instr2_non_ALU_M));
    register #(1,0) non_ALU_M2W_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(instr2_non_ALU_M), .Q(instr2_non_ALU_W));
    
    register #(1,0) single_F2D_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(brn_flush_E), 
            .D(/*~(instr_move_DE & ~brn_state_D[1]) & is_branch_F[0] & brn_state_F[1]*/1'b0), .Q(instr2_single_D));
    register #(1,0) single_D2E_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(brn_flush_E), 
            .D(instr2_single_D), .Q(instr2_single_E));
    register #(1,0) single_E2M_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(brn_flush_E | bcond_E), .Q(instr2_single_M));
    register #(1,0) single_M2W_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(instr2_single_M), .Q(instr2_single_W));
    
    // 0 Instruction Reason
    register #(1,0) stall_D2E_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(brn_flush_E), 
            .D(instr_stall_DE), .Q(instr_stall_E));
    register #(1,0) stall_E2M_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(instr_stall_E), .Q(instr_stall_M));
    register #(1,0) stall_M2W_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(instr_stall_M), .Q(instr_stall_W));
    
    register #(1,0) flushed_F2D_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(brn_flush_E), .Q(flushed_D));
    register #(1,0) flushed_D2E_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(flushed_D | brn_flush_E), .Q(flushed_E));
    register #(1,0) flushed_E2M_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(flushed_E), .Q(flushed_M));
    register #(1,0) flushed_M2W_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(flushed_M), .Q(flushed_W));
    
    // BTB Hit
    register #(1,0) hit_F2D_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(brn_flush_E), 
            .D(hit_F), .Q(hit_D));
    register #(1,0) hit_D2E_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(brn_flush_E), 
            .D(hit_D), .Q(hit_E));
    register #(1,0) hit_E2M_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(hit_E), .Q(hit_M));
    register #(1,0) hit_M2W_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(hit_M), .Q(hit_W));
    
    // Opcode
    opcd_reg #(1) op_E2M_Register(.clk, .rst_l, .en(~halted), .clear(1'b0),
            .D(opcode_E[0]), .Q(opcode_M[0]));
    opcd_reg #(1) op_E2M_Register2(.clk, .rst_l, .en(~halted), .clear(brn_flush_E | bcond_E),
            .D(opcode_E[1]), .Q(opcode_M[1]));
    opcd_reg #(2) op_M2W_Register(.clk, .rst_l, .en(~halted), .clear(1'b0),
            .D(opcode_M), .Q(opcode_W));

    // PC
    register #($bits(pc_E),0) pc_E2M_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(pc_E), .Q(pc_M));
    register #($bits(pc_M),0) pc_M2W_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(pc_M), .Q(pc_W));
    
    genvar k;
    generate
        for (k=0; k<2; k++) begin
            assign is_BRANCH_W[k] = (opcode_W[k] == OP_BRANCH);
            assign is_JAL_W[k] = (opcode_W[k] == OP_JAL);
            assign is_JALR_W[k] = (opcode_W[k] == OP_JALR);
            assign is_OP_W[k] = (opcode_W[k] == OP_OP);
            assign is_IMM_W[k] = (opcode_W[k] == OP_IMM);
            assign is_LUI_W[k] = (opcode_W[k] == OP_LUI);
            assign is_AUIPC_W[k] = (opcode_W[k] == OP_AUIPC);
            assign is_LOAD_W[k] = (opcode_W[k] == OP_LOAD);
            assign is_STORE_W[k] = (opcode_W[k] == OP_STORE);
            assign is_SYSTEM_W[k] = (opcode_W[k] == OP_SYSTEM);
        end
    endgenerate
    
    // Immediate Value
    register #($bits(imm_val_D),0) imm_E2M_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(imm_val_E), .Q(imm_val_M));
    register #($bits(imm_val_D),0) imm_M2W_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(imm_val_M), .Q(imm_val_W));
    
    // Brn State
    register #($bits(brn_state_D),0) brn_E2M_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(brn_state_E), .Q(brn_state_M));
    register #($bits(brn_state_D),0) brn_M2W_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(brn_state_M), .Q(brn_state_W));
    
    // Rs1
    register #($bits(rs1_D),0) rs1_E2M_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(rs1_E), .Q(rs1_M));
    register #($bits(rs1_D),0) rs1_M2W_Register(.clk, .rst_l,
            .en(~halted), .clear(1'b0), .D(rs1_M), .Q(rs1_W));
    
    // Flush
    register #(1,0) flush_E2M_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(brn_flush_E), .Q(brn_flush_M));
    register #(1,0) flush_M2W_Register (.clk, .rst_l,
            .en(~halted & ~syscall_halt_W), .clear(1'b0), 
            .D(brn_flush_M), .Q(brn_flush_W));

    logic[31:0] cycle_num, instr_num, instr_exe_num, stall_num, stall_one_num, stall_two_num, stall_three_num, move_num, mispred_num;
    logic[31:0] stall_one_num_t, stall_two_num_t, stall_three_num_t, arith_num;
    logic[31:0] BRANCH_num, JAL_num, JALR_num, OP_num, IMM_num, LUI_num, AUIPC_num, LOAD_num, STORE_num, SYSTEM_num;
    logic[31:0] branch_fw_t_hit_re, branch_bw_t_hit_re, branch_fw_nt_hit_re, branch_bw_nt_hit_re, branch_fw_t_miss_re, branch_bw_t_miss_re, branch_fw_nt_miss_re, branch_bw_nt_miss_re, 
                branch_fw_t_hit_n, branch_bw_t_hit_n, branch_fw_nt_hit_n, branch_bw_nt_hit_n, branch_fw_t_miss_n, branch_bw_t_miss_n, branch_fw_nt_miss_n, branch_bw_nt_miss_n;
    logic[31:0] jal_x1_hit_re, jal_nx1_hit_re, jal_x1_miss_re, jal_nx1_miss_re, jal_x1_hit_n, jal_nx1_hit_n, jal_x1_miss_n, jal_nx1_miss_n;
    logic[31:0] jalr_x1_hit_re, jalr_nx1_hit_re, jalr_x1_miss_re, jalr_nx1_miss_re, jalr_x1_hit_n, jalr_nx1_hit_n, jalr_x1_miss_n, jalr_nx1_miss_n;
    logic[31:0] instr2_count, instr1_count, instr1_hazard_count, instr1_non_ALU_count, instr1_single_count, instr0_count, instr0_hazard_count, instr0_lack_count;
    logic stall1_en, stall2_en, stall3_en;
    logic[2:0] cur_stalled_count;

    logic [31:0] branch1F_num, branch1E_num;
    logic [31:0] stall_P11, stall_P12, stall_P22, stall_MUL1, stall_MUL2, stall_LW_2;
    // Individual stall condition signals
    
    perf_counter #(1) cycle_counter (.en(~halted), .clk, .rst_l, .clear(1'b0), .Q(cycle_num));
    perf_counter stall_counter (.en(instr_stall), .clk, .rst_l, .clear(1'b0), .Q(stall_num));
    perf_counter move_counter (.en(instr_move_DE), .clk, .rst_l, .clear(1'b0), .Q(move_num));
    perf_counter mispred_counter (.en(brn_flush_W), .clk, .rst_l, .clear(1'b0), .Q(mispred_num));
    perf_counter branch1F_counter (.en(is_branch_F[1]), .clk, .rst_l, .clear(1'b0), .Q(branch1F_num));
    perf_counter branch1E_counter (.en(ctrl_signals_E[1].pc2RF | (opcode_E[1] == OP_BRANCH)), .clk, .rst_l, .clear(1'b0), .Q(branch1E_num));

    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l)
            instr_num <= 0;
        else if (brn_flush_E) begin
            instr_num <= instr_num + 32'd2;
        end 
        else if (instr_move_DE) begin
            instr_num <= is_branch_F[0] ? instr_num : instr_num + 32'd1;
        end
        else if (is_branch_F[0]) begin
            instr_num <= is_branch_F[1] ? (brn_state_F[1] ? 
                    instr_num + 32'd2 : instr_num + 32'd1)
                    : instr_num + 32'd2;
        end
        else if (is_branch_F[1]) begin
            instr_num <= instr_num + 32'd1;
        end
        else begin
            instr_num <= instr_num + 32'd2;
        end
    end

    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l)
            instr_exe_num <= 1;
        else if (~syscall_halt_W & ~invalid_W & invalid_W2)
            instr_exe_num <= instr_exe_num + 1;
        else if (~syscall_halt_W & ~invalid_W & ~invalid_W2)
            instr_exe_num <= instr_exe_num + 2;
    end


    perf_counter stall_one_counter (.en(~halted & stall1_en), .clk, .rst_l, .clear(1'b0), .Q(stall_one_num_t));
    perf_counter stall_two_counter (.en(~halted & stall2_en), .clk, .rst_l, .clear(1'b0), .Q(stall_two_num_t));
    perf_counter stall_three_counter (.en(~halted & stall3_en), .clk, .rst_l, .clear(1'b0), .Q(stall_three_num_t));
    
    perf_counter instr2_counter (.en(~syscall_halt_W && ~invalid_ID && ~invalid_ID2), .clk, .rst_l, .clear(1'b0), .Q(instr2_count));
    perf_counter #(1) instr1_counter (.en(~syscall_halt_W && ~invalid_ID && invalid_ID2), .clk, .rst_l, .clear(1'b0), .Q(instr1_count));
    perf_counter instr1_hazard_counter (.en(~syscall_halt_W && ~invalid_ID && invalid_ID2 && instr2_hazard_D && ~(instr2_non_ALU_D | instr2_non_ALU_FD)), .clk, .rst_l, .clear(1'b0), .Q(instr1_hazard_count));
    perf_counter instr1_non_ALU_counter (.en(~syscall_halt_W && ~invalid_ID && invalid_ID2 && (instr2_non_ALU_D | instr2_non_ALU_FD)), .clk, .rst_l, .clear(1'b0), .Q(instr1_non_ALU_count));
    perf_counter #(1) instr1_single_counter (.en(~syscall_halt_W && ~invalid_ID && invalid_ID2 && instr2_single_D), .clk, .rst_l, .clear(1'b0), .Q(instr1_single_count));
    perf_counter instr0_counter (.en(~syscall_halt_W && invalid_ID && invalid_ID2), .clk, .rst_l, .clear(1'b0), .Q(instr0_count));
    perf_counter instr0_hazard_counter (.en(~syscall_halt_W && invalid_ID && invalid_ID2 && instr_stall_DE), .clk, .rst_l, .clear(1'b0), .Q(instr0_hazard_count));
    perf_counter #(1) instr0_lack_counter (.en(~syscall_halt_W && invalid_ID && invalid_ID2 && (flushed_D | brn_flush_E)), .clk, .rst_l, .clear(1'b0), .Q(instr0_lack_count));
    
    cycle_stalled_record CSR (.*);
    mux #(2, 32) stall_one_last (.in({stall_one_num_t + (cur_stalled_count == 3'd1), stall_one_num_t}), .out(stall_one_num), .sel(halted));
    mux #(2, 32) stall_two_last (.in({stall_two_num_t + (cur_stalled_count == 3'd2), stall_two_num_t}), .out(stall_two_num), .sel(halted));
    mux #(2, 32) stall_three_last (.in({stall_three_num_t + (cur_stalled_count == 3'd3), stall_three_num_t}), .out(stall_three_num), .sel(halted));
    
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l)
            BRANCH_num <= 32'd0;
        else if (is_BRANCH_W[0] & is_BRANCH_W[1])
            BRANCH_num <= BRANCH_num + 32'd2;
        else if (is_BRANCH_W[0] ^ is_BRANCH_W[1])
            BRANCH_num <= BRANCH_num + 32'd1;
    end
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l)
            JAL_num <= 32'd0;
        else if (is_JAL_W[0] & is_JAL_W[1])
            JAL_num <= JAL_num + 32'd2;
        else if (is_JAL_W[0] ^ is_JAL_W[1])
            JAL_num <= JAL_num + 32'd1;
    end
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l)
            JALR_num <= 32'd0;
        else if (is_JALR_W[0] & is_JALR_W[1])
            JALR_num <= JALR_num + 32'd2;
        else if (is_JALR_W[0] ^ is_JALR_W[1])
            JALR_num <= JALR_num + 32'd1;
    end
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l)
            OP_num <= 32'd0;
        else if (is_OP_W[0] & is_OP_W[1])
            OP_num <= OP_num + 32'd2;
        else if (is_OP_W[0] ^ is_OP_W[1])
            OP_num <= OP_num + 32'd1;
    end
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l)
            IMM_num <= 32'd0;
        else if ((is_IMM_W[0] & ~invalid_W) & (is_IMM_W[1] & ~invalid_W2))
            IMM_num <= IMM_num + 32'd2;
        else if ((is_IMM_W[0] & ~invalid_W) ^ (is_IMM_W[1] & ~invalid_W2))
            IMM_num <= IMM_num + 32'd1;
    end
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l)
            LUI_num <= 32'd0;
        else if (is_LUI_W[0] & is_LUI_W[1])
            LUI_num <= LUI_num + 32'd2;
        else if (is_LUI_W[0] ^ is_LUI_W[1])
            LUI_num <= LUI_num + 32'd1;
    end
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l)
            AUIPC_num <= 32'd0;
        else if (is_AUIPC_W[0] & is_AUIPC_W[1])
            AUIPC_num <= AUIPC_num + 32'd2;
        else if (is_AUIPC_W[0] ^ is_AUIPC_W[1])
            AUIPC_num <= AUIPC_num + 32'd1;
    end
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l)
            LOAD_num <= 32'd0;
        else if (is_LOAD_W[0] & is_LOAD_W[1])
            LOAD_num <= LOAD_num + 32'd2;
        else if (is_LOAD_W[0] ^ is_LOAD_W[1])
            LOAD_num <= LOAD_num + 32'd1;
    end
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l)
            STORE_num <= 32'd0;
        else if (is_STORE_W[0] & is_STORE_W[1])
            STORE_num <= STORE_num + 32'd2;
        else if (is_STORE_W[0] ^ is_STORE_W[1])
            STORE_num <= STORE_num + 32'd1;
    end
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l)
            SYSTEM_num <= 32'd0;
        else if (is_SYSTEM_W[0] & is_SYSTEM_W[1])
            SYSTEM_num <= SYSTEM_num + 32'd2;
        else if (is_SYSTEM_W[0] ^ is_SYSTEM_W[1])
            SYSTEM_num <= SYSTEM_num + 32'd1;
    end
    
    perf_counter br_p1  (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & ~imm_val_W[0][31] & brn_state_W[1] & hit_W & brn_flush_W & ~invalid_W), .Q(branch_fw_t_hit_re), .*);
    perf_counter br_p2  (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & imm_val_W[0][31]  & brn_state_W[1] & hit_W & brn_flush_W & ~invalid_W), .Q(branch_bw_t_hit_re), .*);
    perf_counter br_p3  (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & ~imm_val_W[0][31] & ~brn_state_W[1] & hit_W & brn_flush_W & ~invalid_W), .Q(branch_fw_nt_hit_re), .*);
    perf_counter br_p4  (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & imm_val_W[0][31]  & ~brn_state_W[1] & hit_W & brn_flush_W & ~invalid_W), .Q(branch_bw_nt_hit_re), .*);
    perf_counter br_p5  (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & ~imm_val_W[0][31] & brn_state_W[1] & ~hit_W & brn_flush_W & ~invalid_W), .Q(branch_fw_t_miss_re), .*);
    perf_counter br_p6  (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & imm_val_W[0][31]  & brn_state_W[1] & ~hit_W & brn_flush_W & ~invalid_W), .Q(branch_bw_t_miss_re), .*);
    perf_counter br_p7  (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & ~imm_val_W[0][31] & ~brn_state_W[1] & ~hit_W & brn_flush_W & ~invalid_W), .Q(branch_fw_nt_miss_re), .*);
    perf_counter br_p8  (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & imm_val_W[0][31]  & ~brn_state_W[1] & ~hit_W & brn_flush_W & ~invalid_W), .Q(branch_bw_nt_miss_re), .*);
    perf_counter br_p9  (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & ~imm_val_W[0][31] & brn_state_W[1] & hit_W & ~brn_flush_W & ~invalid_W), .Q(branch_fw_t_hit_n), .*);
    perf_counter br_p10 (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & imm_val_W[0][31]  & brn_state_W[1] & hit_W & ~brn_flush_W & ~invalid_W), .Q(branch_bw_t_hit_n), .*);
    perf_counter br_p11 (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & ~imm_val_W[0][31] & ~brn_state_W[1] & hit_W & ~brn_flush_W & ~invalid_W), .Q(branch_fw_nt_hit_n), .*);
    perf_counter br_p12 (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & imm_val_W[0][31]  & ~brn_state_W[1] & hit_W & ~brn_flush_W & ~invalid_W), .Q(branch_bw_nt_hit_n), .*);
    perf_counter br_p13 (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & ~imm_val_W[0][31] & brn_state_W[1] & ~hit_W & ~brn_flush_W & ~invalid_W), .Q(branch_fw_t_miss_n), .*);
    perf_counter br_p14 (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & imm_val_W[0][31]  & brn_state_W[1] & ~hit_W & ~brn_flush_W & ~invalid_W), .Q(branch_bw_t_miss_n), .*);
    perf_counter br_p15 (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & ~imm_val_W[0][31] & ~brn_state_W[1] & ~hit_W & ~brn_flush_W & ~invalid_W), .Q(branch_fw_nt_miss_n), .*);
    perf_counter br_p16 (.clear(1'b0), .en(~halted & is_BRANCH_W[0] & imm_val_W[0][31]  & ~brn_state_W[1] & ~hit_W & ~brn_flush_W & ~invalid_W), .Q(branch_bw_nt_miss_n), .*);
    
    perf_counter jal_p1 (.clear(1'b0), .en(~halted & is_JAL_W[0] & (rd_W[0] == 1) & hit_W & brn_flush_W & ~invalid_W), .Q(jal_x1_hit_re), .*);
    perf_counter jal_p2 (.clear(1'b0), .en(~halted & is_JAL_W[0] & (rd_W[0] != 1) & hit_W & brn_flush_W & ~invalid_W), .Q(jal_nx1_hit_re), .*);
    perf_counter jal_p3 (.clear(1'b0), .en(~halted & is_JAL_W[0] & (rd_W[0] == 1) & ~hit_W & brn_flush_W & ~invalid_W), .Q(jal_x1_miss_re), .*);
    perf_counter jal_p4 (.clear(1'b0), .en(~halted & is_JAL_W[0] & (rd_W[0] != 1) & ~hit_W & brn_flush_W & ~invalid_W), .Q(jal_nx1_miss_re), .*);
    perf_counter jal_p5 (.clear(1'b0), .en(~halted & is_JAL_W[0] & (rd_W[0] == 1) & hit_W & ~brn_flush_W & ~invalid_W), .Q(jal_x1_hit_n), .*);
    perf_counter jal_p6 (.clear(1'b0), .en(~halted & is_JAL_W[0] & (rd_W[0] != 1) & hit_W & ~brn_flush_W & ~invalid_W), .Q(jal_nx1_hit_n), .*);
    perf_counter jal_p7 (.clear(1'b0), .en(~halted & is_JAL_W[0] & (rd_W[0] == 1) & ~hit_W & ~brn_flush_W & ~invalid_W), .Q(jal_x1_miss_n), .*);
    perf_counter jal_p8 (.clear(1'b0), .en(~halted & is_JAL_W[0] & (rd_W[0] != 1) & ~hit_W & ~brn_flush_W & ~invalid_W), .Q(jal_nx1_miss_n), .*);
    
    perf_counter jalr_p1 (.clear(1'b0), .en(~halted & is_JALR_W[0] & (rs1_W[0] == 1) & hit_W & brn_flush_W & ~invalid_W), .Q(jalr_x1_hit_re), .*);
    perf_counter jalr_p2 (.clear(1'b0), .en(~halted & is_JALR_W[0] & (rs1_W[0] != 1) & hit_W & brn_flush_W & ~invalid_W), .Q(jalr_nx1_hit_re), .*);
    perf_counter jalr_p3 (.clear(1'b0), .en(~halted & is_JALR_W[0] & (rs1_W[0] == 1) & ~hit_W & brn_flush_W & ~invalid_W), .Q(jalr_x1_miss_re), .*);
    perf_counter jalr_p4 (.clear(1'b0), .en(~halted & is_JALR_W[0] & (rs1_W[0] != 1) & ~hit_W & brn_flush_W & ~invalid_W), .Q(jalr_nx1_miss_re), .*);
    perf_counter jalr_p5 (.clear(1'b0), .en(~halted & is_JALR_W[0] & (rs1_W[0] == 1) & hit_W & ~brn_flush_W & ~invalid_W), .Q(jalr_x1_hit_n), .*);
    perf_counter jalr_p6 (.clear(1'b0), .en(~halted & is_JALR_W[0] & (rs1_W[0] != 1) & hit_W & ~brn_flush_W & ~invalid_W), .Q(jalr_nx1_hit_n), .*);
    perf_counter jalr_p7 (.clear(1'b0), .en(~halted & is_JALR_W[0] & (rs1_W[0] == 1) & ~hit_W & ~brn_flush_W & ~invalid_W), .Q(jalr_x1_miss_n), .*);
    perf_counter jalr_p8 (.clear(1'b0), .en(~halted & is_JALR_W[0] & (rs1_W[0] != 1) & ~hit_W & ~brn_flush_W & ~invalid_W), .Q(jalr_nx1_miss_n), .*);
    
    /****** Performance Recording ******/

`endif /* SIMULATION_18447 */

    /* When the design is compiled for simulation, the Makefile defines
     * SIMULATION_18447. You can use this to have code that is there for
     * simulation, but is discarded when the design is synthesized. Useful
     * for constructs that can't be synthesized. */
`ifdef SIMULATION_18447
`ifdef TRACE

    // Extract different instruction fields
    // Only for primary stage instruction
    opcode_t opcode;
    funct7_t funct7;
    rtype_funct3_t rtype_funct3;
    itype_int_funct3_t itype_int_funct3;
    assign opcode           = opcode_t'(instr_D[0][6:0]);
    assign funct7           = funct7_t'(instr_D[0][31:25]);
    assign rtype_funct3     = rtype_funct3_t'(instr_D[0][14:12]);
    assign itype_int_funct3 = itype_int_funct3_t'(instr_D[0][14:12]);

    /* Cycle-by-cycle trace messages. You'll want to comment this out for
     * longer tests, or they will take much, much longer to run. Be sure to
     * comment this out before submitting your code, so tests can be run
     * quickly. */

    logic [31:0] fibr_count;
    always_ff @(posedge clk) begin
        if (~rst_l)
            fibr_count <= 32'd0;
        else if (pc_W[0] == 32'h4004ec)
            fibr_count <= fibr_count + 32'd1;
    end


    function void print_regs ();
        for (int i=0; i<16; i++) begin
            $display("\t x%2d: %h\tx%2d: %x", i, S2.Reg_File.registers[i], i+16, S2.Reg_File.registers[i+16]);
        end

    endfunction: print_regs
    
    always_ff @(posedge clk) begin
        if (rst_l) begin
            // if (pc_W[0] == 32'h4004ec) begin
            //     // $display("\t Cycle num: %d", cycle_count);
            //     $display("Called fibr*********************************");
            //     $display("Called from:  %h", S2.Reg_File.registers[1] - 32'd4);
            //     $display("FIBR count:   %d", fibr_count);

            //     $display("\t Branch Count: %d", BRANCH_num);
            //     $display("\t JAL Count:    %d", JAL_num);
            //     $display("\t JALR Count:   %d", JALR_num);
            //     $display("\t OP Count:     %d", OP_num);
            //     $display("\t IMM Count:    %d", IMM_num);
            //     $display("\t LUI Count:    %d", LUI_num);
            //     $display("\t AUIPC Count:  %d", AUIPC_num);
            //     $display("\t LOAD Count:   %d", LOAD_num);
            //     $display("\t STORE Count:  %d", STORE_num);
            //     $display("\t SYSTEM Count: %d", SYSTEM_num);
            //     $display("\t Added count:  %d", BRANCH_num + JAL_num + JALR_num
            //             + OP_num + IMM_num + LUI_num + AUIPC_num + LOAD_num
            //             + STORE_num + SYSTEM_num);
            //     $display();
            //     print_regs();
            //     $display("\n");
            //     $display("Total instrs: %d\n", instr_exe_num);
            // end
            /*
            $display("Instr0: 0x%08x", instr[0]);
            $display("Instr1: 0x%08x", instr[1]);
            $display("Instr2: 0x%08x\n", instr[2]);

            $display("\n- Simulation Cycle %0d", $time);
            $display({80{"-"}}, "\n");
            $display("\tPC: 0x%08x, 0x%08x\n", pc_F[0], pc_F[1]);
            $display("\tclear %d\n", clear_I2_F);
            $display("\tPC_D: 0x%08x, 0x%08x\n", pc_D[0], pc_D[1]);
            
            $display("\tPC_E: 0x%08x, 0x%08x\n", pc_E[0], pc_E[1]);
            $display("\trs1: %d, %d rs2: %d %d\n", rs1_E[0], rs1_E[1], rs2_E[0], rs2_E[1]);
            $display("\ts1: 0x%08x 0x%08x s2: 0x%08x 0x%08x\n", rs1_fwd_data_DEMW[0], rs1_fwd_data_DEMW[1], rs2_fwd_data_DEMW[0], rs2_fwd_data_DEMW[1]);
            $display("\tTarget: 0x%08x, Pred: 0x%08x, ALU: 0x%08x, %d\n", brn_tgt_E, pred_pc_E, brn_taken_E, brn_flush_E);
            $display("\trd_data_E: 0x%08x, 0x%08x\n", rd_data_E[0], rd_data_E[1]);
            
            //$display("\tPC_M: 0x%08x, 0x%08x\n", pc_M[0], pc_M[1]);
            $display("\trd_M: %d, %d\n", rd_M[0], rd_M[1]);
            $display("\tALU_M: 0x%08x, 0x%08x\n", alu_out_M[0], alu_out_M[1]);
            
            //$display("\tPC_W: 0x%08x, 0x%08x\n", pc_W[0], pc_W[1]);
            $display("\trd_W: %d, %d\n", rd_W[0], rd_W[1]);
            $display("\trd_data_W: 0x%08x, 0x%08x\n", rd_data_W[0], rd_data_W[1]);
            
            /*$display({"\n", {80{"-"}}});
            $display("- Simulation Cycle %0d", $time);
            $display({{80{"-"}}, "\n"});

            $display("\tPC: 0x%08x", pc_F);
            $display("\tInstructions: 0x%08x, 0x%08x\n", instr_F, instr_D);

            $display("\tInstruction Memory Exception: %0b", instr_mem_excpt);
            $display("\tData Memory Exception: %0b", data_mem_excpt);
            $display("\tIllegal Instruction Exception: %0b", ctrl_signals_D.illegal_instr);
            $display("\tStalled: %0b, Reach End: %d\n", instr_stall, syscall_halt_W);
            $display("\tHalted: %0b\n", halted);

            $display("\tDecode reg:  %0d, %0d, %0d, %0d, %0d", rd_D, rs1_D, rs1_data_D, rs2_D, rs2_data_D);
            $display("\tForward reg:  %0d, %0d", rs1_fwd_data_DEMW, rs2_fwd_data_DEMW);
            $display("\tExecute reg:   %0d, out %0d, mul %d", rd_E, alu_out_E, mult_out_E);
            $display("\tMemory reg:    %0d  read %d addr %d store %d", rd_M, data_load_M, data_addr_full_M, data_store);
            $display("\tWriteback reg: %0d", rd_W);
            $display("\tRFWrite DEMW %4b", {ctrl_signals_D.rfWrite, ctrl_signals_E.rfWrite,
                    ctrl_signals_M.rfWrite, ctrl_signals_W.rfWrite});*/
                    
            

            /*$display("\tOpcode: 0x%02x (%s)", opcode, opcode.name);
            $display("\tFunct3: 0x%01x (%s | %s)", rtype_funct3, rtype_funct3.name, itype_int_funct3.name);
            $display("\tFunct7: 0x%02x (%s)", funct7, funct7.name);
            $display("\trs1: %0d", rs1_D);
            $display("\trs2: %0d", rs2_D);
            $display("\trd: %0d", rd_D);
            //$display("\tSign Extended Immediate: %0d", S2.se_immediate);
            $display("\tRegSel rs1d %d rs2d %d rdw %d\n", rs1_D, rs2_D, rd_W);
            $display("\tRegfile %d %d %d\n", rs1_data_D, rs2_data_D, rd_data_W);
            $display("\tAlu_out %d Alu_op %s src1 %d src2 %d\n", alu_out_E, ctrl_signals_E.alu_op.name, rs1_data_E, S3.alu_src2);
            $display("\tAddr %h, Write %h, Read %h, Mask %b\n", data_addr, data_store, data_load, data_store_mask);
            $display("\tRegfile write is %d, npc+4d = %d, pc2RFw = %d\n", ctrl_signals_W.rfWrite, npc_plus4_W, ctrl_signals_W.pc2RF);
            $display("\t");*/
        end
        
    
        if (halted) begin
            $display("\n\t********* General Performance **********\n");
            $display("\t Cycle Count: %d", cycle_num);
            $display("\t Stall Count: %d", stall_num);
            $display("\t Move Count: %d", move_num);
            $display("\t Mispredicts: %d", mispred_num);
            

            $display("\t Instr Fetched Count: %d", instr_num);
            $display("\t Instr Executed: %d", instr_exe_num);
            $display("\t 2 Instr Issue: %d", instr2_count);
            $display("\t 1 Instr Issue: %d", instr1_count);
            $display("\t 1 Instr Issue due to hazard: %d", instr1_hazard_count);
            $display("\t 1 Instr Issue due to non ALU: %d", instr1_non_ALU_count);
            $display("\t 1 Instr Issue due to single: %d", instr1_single_count);
            $display("\t 0 Instr Issue: %d", instr0_count);
            $display("\t 0 Instr Issue due to hazard: %d", instr0_hazard_count);
            $display("\t 0 Instr Issue due to lack of instr: %d\n", instr0_lack_count);
            
            $display("\t Branch Count: %d", BRANCH_num);
            $display("\t JAL Count:    %d", JAL_num);
            $display("\t JALR Count:   %d", JALR_num);
            $display("\t OP Count:     %d", OP_num);
            $display("\t IMM Count:    %d", IMM_num);
            $display("\t LUI Count:    %d", LUI_num);
            $display("\t AUIPC Count:  %d", AUIPC_num);
            $display("\t LOAD Count:   %d", LOAD_num);
            $display("\t STORE Count:  %d", STORE_num);
            $display("\t SYSTEM Count: %d", SYSTEM_num);
            $display("\t Added count:  %d", BRANCH_num + JAL_num + JALR_num
                        + OP_num + IMM_num + LUI_num + AUIPC_num + LOAD_num
                        + STORE_num + SYSTEM_num);

            /*$display("\t ALU Count: %d\n", ALU_num);
            $display("\t LOAD Count: %d\n", LW_num);
            $display("\t STORE Count: %d\n", SW_num);
            $display("\t SYS Count: %d\n", ECALL_num);*/
            
            //$display("\t  Branch P1F: %d\n", branch1F_num);
            //$display("\t  Branch P1E: %d\n", branch1E_num);
            
            /*$display("\t********* Branch Performance **********\n");
            $display("\t Branch Forward Taken Hit Rewind: %d ", branch_fw_t_hit_re);
            $display("\t Branch Backward Taken Hit Rewind: %d ", branch_bw_t_hit_re);
            $display("\t Branch Forward Not Taken Hit Rewind: %d ", branch_fw_nt_hit_re);
            $display("\t Branch Backward Not Taken Hit Rewind: %d ", branch_bw_nt_hit_re);
            $display("\t Branch Forward Taken Miss Rewind: %d ", branch_fw_t_miss_re);
            $display("\t Branch Backward Taken Miss Rewind: %d ", branch_bw_t_miss_re);
            $display("\t Branch Forward Not Taken Miss Rewind: %d ", branch_fw_nt_miss_re);
            $display("\t Branch Backward Not Taken Miss Rewind: %d ", branch_bw_nt_miss_re);
            $display("\t Branch Forward Taken Hit No Rewind: %d ", branch_fw_t_hit_n);
            $display("\t Branch Backward Taken Hit No Rewind: %d ", branch_bw_t_hit_n);
            $display("\t Branch Forward Not Taken Hit No Rewind: %d ", branch_fw_nt_hit_n);
            $display("\t Branch Backward Not Taken Hit No Rewind: %d ", branch_bw_nt_hit_n);
            $display("\t Branch Forward Taken Miss No Rewind: %d ", branch_fw_t_miss_n);
            $display("\t Branch Backward Taken Miss No Rewind: %d ", branch_bw_t_miss_n);
            $display("\t Branch Forward Not Taken Miss No Rewind: %d ", branch_fw_nt_miss_n);
            $display("\t Branch Backward Not Taken Miss No Rewind: %d ", branch_bw_nt_miss_n);
            $display("\n");
            
            $display("\t********* JAL Performance **********\n");
            $display("\t JAL x1 Hit Rewind: %d ", jal_x1_hit_re);
            $display("\t JAL not x1 Hit Rewind: %d ", jal_nx1_hit_re);
            $display("\t JAL x1 Miss Rewind: %d ", jal_x1_miss_re);
            $display("\t JAL not x1 Miss Rewind: %d ", jal_nx1_miss_re);
            $display("\t JAL x1 Hit No Rewind: %d ", jal_x1_hit_n);
            $display("\t JAL not x1 Hit No Rewind: %d ", jal_nx1_hit_n);
            $display("\t JAL x1 Miss No Rewind: %d ", jal_x1_miss_n);
            $display("\t JAL not x1 Miss No Rewind: %d ", jal_nx1_miss_n);
            $display("\n");
            
            $display("\t********* JALR Performance **********\n");
            $display("\t JALR x1 Hit Rewind: %d ", jalr_x1_hit_re);
            $display("\t JALR not x1 Hit Rewind: %d ", jalr_nx1_hit_re);
            $display("\t JALR x1 Miss Rewind: %d ", jalr_x1_miss_re);
            $display("\t JALR not x1 Miss Rewind: %d ", jalr_nx1_miss_re);
            $display("\t JALR x1 Hit No Rewind: %d ", jalr_x1_hit_n);
            $display("\t JALR not x1 Hit No Rewind: %d ", jalr_nx1_hit_n);
            $display("\t JALR x1 Miss No Rewind: %d ", jalr_x1_miss_n);
            $display("\t JALR not x1 Miss No Rewind: %d ", jalr_nx1_miss_n);
            $display("\n");
            */
        end
    end
    

`endif /* TRACE */
`endif /* SIMULATION_18447 */

endmodule: riscv_core



/**
 * The arithmetic-logic unit (ALU) for the RISC-V processor. Located in E-stage 
 *
 * The ALU handles executing the current instruction, producing the
 * appropriate output based on the ALU operation specified to it by the
 * decoder.
 *
 * Inputs:
 *  - alu_src1      The first operand to the ALU.
 *  - alu_src2      The second operand to the ALU.
 *  - alu_op        The ALU operation to perform.
 * Outputs:
 *  - alu_out       The result of the ALU operation on the two sources.
 **/
module riscv_alu
    (input  logic [31:0]    alu_src1,
     input  logic [31:0]    alu_src2,
     input  alu_op_t        alu_op,
     output logic [31:0]    alu_out);

    // Internal signals for operations that take more than 1 line
    logic [31:0] sum, slt, sra, temp;

    // Adder for ADD and SUB
    adder #($bits(alu_src1)) ALU_Adder(.A(alu_src1),
                                       .B((alu_op==ALU_SUB)?(~alu_src2):alu_src2),
                                       .cin(alu_op==ALU_SUB),
                                       .sum, .cout());

    // Perform signed comparison for SLT
    always_comb begin
        if (alu_src1[31] == alu_src2[31])
            slt = (alu_src1 < alu_src2);
        else if (alu_src1[31] == 1'b1)
            slt = 32'd1;
        else
            slt = 32'd0;
    end

    // Perform arithmetic right-shift (max of 31 shifts possible)
    always_comb begin
        temp = ~(32'hffff_ffff >> (alu_src2[4:0]));     // Sign extension mask
        if (alu_src1[31] == 1'b1)
            sra = (alu_src1 >> alu_src2[4:0]) | temp;
        else
            sra = alu_src1 >> alu_src2[4:0];
    end

    // Case on all possible operations
    always_comb begin
        unique case (alu_op)
            ALU_ADD: alu_out = sum;
            ALU_SUB: alu_out = sum;
            ALU_SLL: alu_out = alu_src1 << {27'd0, alu_src2[4:0]};
            ALU_SLT: alu_out = slt;
            ALU_SLTU: alu_out = (alu_src1 < alu_src2) ? 32'd1 : 32'd0;
            ALU_XOR: alu_out = alu_src1 ^ alu_src2;
            ALU_SRL: alu_out = alu_src1 >> {27'd0, alu_src2[4:0]};
            ALU_SRA: alu_out = sra;
            ALU_OR: alu_out = alu_src1 | alu_src2;
            ALU_AND: alu_out = alu_src1 & alu_src2;
            default: alu_out = 'bx;
        endcase
    end

endmodule: riscv_alu



/**
 * Module that given the instruction and type, gives out the corresponding
 * immediate value. Located in the D stage
 *
 * Inputs:
 * - instruction    The raw 32-bit current instruction
 * - ctrl_signals   The control signals from the decoder
 *
 * Outputs:
 * - imm            The interpreted immediate value based on instruction format
 **/
module riscv_imm_decode
    (input  logic [31:0] instruction,
     input  ctrl_signals_t  ctrl_signals,
     output logic [31:0] imm);

    always_comb begin
        case (ctrl_signals.imm_mode)
            IMM_I: begin
                if (instruction[6:0] == 7'b0010011 && instruction[13:12] == 2'b01)
                    imm = {27'd0, instruction[24:20]};
                else
                    imm = {{20{instruction[31]}}, instruction[31:20]};
            end

            IMM_S:
                imm = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};

            IMM_SB:
                imm = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};

            IMM_U:
                imm = {instruction[31:12], 12'd0};

            IMM_UJ:
                imm = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};

            default: imm = 32'd0;
        endcase
    end

endmodule: riscv_imm_decode



/**
 * Resolves branches by comparing predicted nextPC to actual nextPC. The updated
 * PC and branch taken/not taken is fed back to the BTB in the F stage. Located
 * in the E stage
 *
 * Inputs:
 * - alu_output     The output from a SUB operation of rs1_data - rs2_data
 * - sign1          The sign of rs1_data
 * - sign2          The sign of rs2_data
 * - ctrl_signals   Control signals are used to figure out the branch type
 *
 * Outputs:
 * - bcond          Single-bit signal to indicate use PC+4 or PC+imm for SB-type
 **/
module riscv_branch_cond
    (input  logic [31:0]    alu_output,
     input  logic           sign1, sign2,
     input  ctrl_signals_t  ctrl_signals,
     output logic           bcond);

    import RISCV_ISA::*;

    always_comb begin
        case (ctrl_signals.btype)
            FUNCT3_BEQ: bcond = (alu_output == 32'd0);
            FUNCT3_BNE: bcond = (alu_output != 32'd0);
            FUNCT3_BLT: bcond = (sign1 && ~sign2) || (sign1 == sign2 && alu_output[31] == 1'b1);
            FUNCT3_BGE: bcond = (~sign1 && sign2) || (sign1 == sign2 && alu_output[31] == 1'b0);
            FUNCT3_BLTU: bcond = (sign2 && ~sign1) || (sign1 == sign2 && alu_output[31] == 1'b1);
            FUNCT3_BGEU: bcond = (~sign2 && sign1) || (sign1 == sign2 && alu_output[31] == 1'b0);
            default: bcond = 1'b0;
        endcase
    end

endmodule: riscv_branch_cond


/**
 * Computes the data to write into memory and the position it should be placed
 * based on control signals and an input position.
 *
 * Inputs:
 * - data_to_write  The raw 32-bit data to be written
 * - pos            Based on address to store at
 * - ctrl_signals   A copy of the control signals from the decoder
 *
 * Outputs:
 * - data_store     The processed data to actually be written
 * - data_store_mask Byte-enabled bit mask to select which bytes of data_store to write
 **/
module riscv_mem_write_ctrl
    (input  logic [31:0] data_to_write,
     input  logic [1:0]  pos,
     input  ctrl_signals_t  ctrl_signals,
     output logic [31:0] data_store,
     output logic [3:0] data_store_mask);

    // Import ISA types and parameters
    import RISCV_ISA::*;

    // Handle memory writes
    always_comb begin
        case (ctrl_signals.ldst_mode)
            // For byte loading, place in correct location based on target addr
            LDST_B: begin
                if (pos == 2'd0) begin
                    data_store = data_to_write[7:0];
                    data_store_mask = 4'b0001;
                end
                else if (pos == 2'd1) begin
                    data_store = {data_to_write[7:0], 8'd0};
                    data_store_mask = 4'b0010;
                end
                else if (pos == 2'd2) begin
                    data_store = {data_to_write[7:0], 16'd0};
                    data_store_mask = 4'b0100;
                end
                else begin
                    data_store = {data_to_write[7:0], 24'd0};
                    data_store_mask = 4'b1000;
                end
            end

            LDST_H: begin
                if (pos == 2'd0) begin
                    data_store = data_to_write[15:0];
                    data_store_mask = 4'b0011;
                end
                else begin      // Memory addresses are well-formed
                    data_store = {data_to_write[15:0], 16'd0};
                    data_store_mask = 4'b1100;
                end
            end

            // LDST_W
            default: begin
                data_store = data_to_write;
                data_store_mask = 4'b1111;
            end
        endcase
    end

endmodule: riscv_mem_write_ctrl



/**
 * Receives data from memory and formats it as requested by the instruction.
 * Includes all sign-extension logic for LH and LB.
 *
 * Inputs:
 * - data_load      The data read from memory
 * - pos            The position in the word the data was read from
 * - ctrl_signals   A copy of the control signals from the decoder
 *
 * Outputs:
 * - data_recv      The data to be stored in the register file on a memory read
 **/
module riscv_mem_read_ctrl
    (input  logic [31:0] data_load,
     input  logic [1:0]  pos,
     input  ctrl_signals_t  ctrl_signals,
     output logic [31:0] data_recv);

    // Import ISA types and parameters
    import RISCV_ISA::*;

    logic [7:0] byte_read;
    logic [15:0] half_read;
    
    // Handle memory reads
    always_comb begin
        data_recv = 32'd0;
        byte_read = 8'd0;
        half_read = 16'd0;

        if (ctrl_signals.memRead) begin
            case (ctrl_signals.ldst_mode)
                LDST_B: begin
                    case (pos)
                        2'd0: byte_read = data_load[7:0];
                        2'd1: byte_read = data_load[15:8];
                        2'd2: byte_read = data_load[23:16];
                        2'd3: byte_read = data_load[31:24];
                    endcase
                    // Sign-extension
                    data_recv = {{24{byte_read[7]}}, byte_read};
                end

                LDST_H: begin
                    case (pos)
                        2'd0: half_read = data_load[15:0];
                        2'd2: half_read = data_load[31:16];
                        default: half_read = 16'd0;
                    endcase
                    // Sign-extension
                    data_recv = {{16{half_read[15]}}, half_read};
                end

                LDST_W: begin
                    // Take full word with no modifications
                    data_recv = data_load;
                end

                LDST_BU: begin
                    case (pos)
                        2'd0: byte_read = data_load[7:0];
                        2'd1: byte_read = data_load[15:8];
                        2'd2: byte_read = data_load[23:16];
                        2'd3: byte_read = data_load[31:24];
                    endcase
                    data_recv = {24'd0, byte_read};
                end

                LDST_HU: begin
                    case (pos)
                        2'd0: half_read = data_load[15:0];
                        2'd2: half_read = data_load[31:16];
                        default: half_read = 16'h1110;
                    endcase
                    data_recv = {16'd0, half_read};
                end

                default: ;      // Default values already set
            endcase
        end
    end

endmodule: riscv_mem_read_ctrl



/**
 * Module that centralizes writing to the register file, from all possible
 * locations that a register write could originate. Located in the W stage
 *
 * Inputs:
 * - pc             Output of the program counter register (used for AUIPC)
 * - alu_out        ALU output
 * - npc_plus4      PC+4, used for JAL/JALR
 * - data_recv      The data that was read from memory
 * - ctrl_signals   The instruction control signals for selecting data source

 * Outputs:
 * - rd_data        The data to be written to a register
 * - rd_we          Register file write-enable signal
 **/

module riscv_rd_data
    (input  logic [31:0] alu_out, data_recv,
     input  ctrl_signals_t  ctrl_signals,
     output logic [31:0] rd_data,
     output logic rd_we);

    // Import ISA types and parameters
    import RISCV_ISA::*;

    assign rd_we = ctrl_signals.rfWrite;

    mux #(2,32) rd_mux (.in({data_recv,alu_out}), .sel(ctrl_signals.mem2RF),
            .out(rd_data));

endmodule: riscv_rd_data



/**
 * A specialized pipeline register for passing control signals through. Primary
 * benefit is setting the reset value to be a nop with no architectural changes.
 *
 * Inputs:
 * - D              The control signal struct from the prior stage
 * - clk            Processor clock
 * - en             Register enable
 * - rst_l          Asynchronous register reset
 * - clear          Synchronous reset signal

 * Outputs:
 * - Q              The control signal struct into the next stage
 **/
module ctrl_reg 
    #(parameter CW = 3) (
    input  ctrl_signals_t [CW-1:0] D,
    input  logic clk, en, rst_l, clear,
    output ctrl_signals_t [CW-1:0] Q);
    
    import RISCV_ISA::*;
    
    ctrl_signals_t reset_val;
    assign reset_val = NOP_CTRL;
    
    always_ff @(posedge clk, negedge rst_l) begin
         if (!rst_l)
             Q <= {CW{reset_val}};
         else if (clear)
             Q <= {CW{reset_val}};
         else if (en)
             Q <= D;
    end
    
endmodule : ctrl_reg



/**
 * A specialized pipeline register for passing opcodes through. Primary benefit
 * is setting the reset value to be OP_IMM for NOP's, rather than undefined.
 *
 * Inputs:
 * - D              The opcode from the prior stage
 * - clk            Processor clock
 * - en             Register enable
 * - rst_l          Asynchronous register reset
 * - clear          Synchronous reset signal

 * Outputs:
 * - Q              The opcode into the next stage
 **/
module opcd_reg #(W=2) (
    input  opcode_t [W-1:0] D,
    input  logic clk, en, rst_l, clear,
    output opcode_t [W-1:0] Q);
    
    import RISCV_ISA::*;
    
    always_ff @(posedge clk, negedge rst_l) begin
         if (!rst_l)
             Q <= {W{OP_IMM}};
         else if (clear)
             Q <= {W{OP_IMM}};
         else if (en)
             Q <= D;
    end
    
endmodule: opcd_reg



/**
 * A counter for pipeline performance tracking, with a parameterized reset value
 *
 * Inputs:
 * - clk            Processor clock
 * - en             Counter enable
 * - rst_l          Asynchronous counter reset
 * - clear          Synchronous reset signal

 * Outputs:
 * - Q              The counter's value
 **/
module perf_counter 
    #(parameter logic [31:0]    RESET_VAL=31'd0)
    (input logic clk, en, rst_l, clear,
    output logic[31:0] Q);
    
    always_ff @(posedge clk, negedge rst_l) begin
        if (!rst_l)
            Q <= RESET_VAL;
        else if (clear)
            Q <= 0;
        else if (en)
            Q <= Q+1;
    end
    
endmodule : perf_counter



/**
 * A module to set the enable signals for our various stall counters. It sets
 * the enable siganls for our 1, 2, and 3-cycle counters.
 *
 * Inputs:ge
 * - clk            Processor clock
 * - rst_l          Asynchronous system reset
 * - instr_stall    The stall signal to track

 * Outputs:
 * - stall1_en      Enable line for the 1-cycle stall counter
 * - stall2_en      Enable line for the 2-cycle stall counter
 * - stall3_en      Enable line for the 3-cycle stall counter
 * - cur_stalled_count  The number of consecutive stall cycles observed
 **/
module cycle_stalled_record (
    input  logic clk, rst_l, instr_stall,
    output logic stall1_en, stall2_en, stall3_en,
    output logic[2:0] cur_stalled_count);

    logic[2:0] stalled_count;

    always_ff @(posedge clk, negedge rst_l) begin
        if (~rst_l) begin
            stall1_en <= 1'b0;
            stall2_en <= 1'b0;
            stall3_en <= 1'b0;
            stalled_count <= 3'd0;
        end
        else begin
            if (instr_stall) begin
                if (stalled_count == 3'd3) begin
                    stalled_count <= 3'd1;
                    stall3_en <= 1'd1;
                end
                else begin
                    stalled_count <= stalled_count + 3'd1;
                    stall1_en <= 1'b0;
                    stall2_en <= 1'b0;
                    stall3_en <= 1'b0;
                end
            end
            else begin
                // Enable each counter once per stall sequence
                stall1_en <= (stalled_count == 3'd1);
                stall2_en <= (stalled_count == 3'd2);
                stall3_en <= (stalled_count == 3'd3);
                stalled_count <= 3'd0;
            end
        end
    end
    
    assign cur_stalled_count = stalled_count;

endmodule: cycle_stalled_record
