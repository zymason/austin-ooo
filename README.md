[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-8d59dc4de5201274e310e4c54b9627a8934c3b88527886e3b421487c677d23eb.svg)](https://classroom.github.com/a/N3oS-Yh4)
# 18-447 RISC-V Verilog Processor Lab

The formatting in this markdown file is best viewed online in Github.

In addition to this README, please also consult *18-447 Handout #3/Lab 1 Part B* for project requirements and logistics to get started on Lab 1.

This starter project is also used as the basis for subsequent labs.  Please consult their respective project handouts for new project requirements and logisttics. Later when working on Labs 2, 3, and 4, please
refer to the end of this README for instructions specific to the later labs.

## General Overview

This starter project directory contains a partially completed *single-cycle* implementation that is almost ready
to support the `addi`, `add`, `lw`, and `beq` RISC-V instructions.  For Lab 1 Part B, you need to complete the single-cycle
implementation for the same subset of RV32I and MUL of RV32M as in Lab 1 Part A (Handout #2). 
Refer to [The RISC-V Instruction Set Manual, Volume I: User-Level ISA](https://riscv.org/specifications/) for their specifications.

To complete the single-cycle implementation, you need to add support for the missing instructions in the `riscv_core`
module in **[src/riscv_core.sv](src/riscv_core.sv)**.

```systemverilog
module riscv_core
    (input  logic           clk, rst_l, instr_mem_excpt, data_mem_excpt,
     input  logic [31:0]    instr, data_load,
     output logic           data_load_en, halted,
     output logic [3:0]     data_store_mask,
     output logic [29:0]    instr_addr, data_addr,
     output logic [31:0]    data_store);
```

This module contains the RTL description of the processor datapath.  It receives a clock and reset from the
environment. It has interfaces to external program and data memory.  Do not change the interface of this module.
This module is instantiated in the provided environment (using *named port* syntax) for simulation and synthesis.

The so-called “single-cycle” microarchitecture in Lab 1 Part B executes one instruction fully in each cycle before
moving on to the next.  The architectural register state (the program counter and general-purpose registers) are
stored in registers synchronized by a global clock. The architectural memory state are stored in external memory
modules that can be read combinationally and written synchronously (to the same global clock as the registers).
In between clock edges, the output of the architectural state registers are fed into a combinational datapath
consisting of logic gates and memory read path to compute the next state according to the instruction currently
pointed to by the program counter.  “Computing the next state” involves generating both the changed state values
and the corresponding latch enable signals to effect the update on the next rising clock edge.  You should find
the operation of this single-cycle microarchitecture to be conceptually very similar to the architectural simulator
in Part A.

Before you start coding, study `riscv_core` and other modules in **src** to understand the datapath described.
The **src** directory also provides a `riscv_decode` decoder module (**[src/riscv_decode.sv](src/riscv_decode.sv)**),
and few helper library  modules (**[src/lib.sv](src/lib.sv)**).   The very first step is to instantiate and wire up
the missing register file (**[447src/register_file.sv](447src/register_file.sv)**).  The datapath will then be ready to
execute `addi`, `add`, `lw`, and `beq` instructions.

You are free to add, change, and delete any files under
the **src** directory, but you are not allowed to modify any files outside the **src** directory. The build system
will automatically discover any new files you add under the **src** directory, provided that they have a *.sv*,
*.v*, or *.vh* extension. The files may be nested in any subdirectories under the **src** directory. Additionally,
the build system sets up the include paths so that you can place header files in any subdirectory under the **src**
directory as well, and include them from anywhere inside the **src** directory.

The starter project make use of certain constructs,
such as enumerations, structs, and unions, which will make coding the processor much simpler. For a complete guide
to all parts of the SystemVerilog language, see the [IEEE 1800-2012 SystemVerilog Language Reference Manual
(LRM)](http://www.ece.uah.edu/~gaede/cpe526/2012%20System%20Verilog%20Language%20Reference%20Manual.pdf).

Go to [HDLBits](http://hdlbits.01xz.net/) if you like a quick refresher on RTL Verilog.

## Getting Started

### Machines

The build system and code for the 18-447 labs is designed to run on ECE Linux workstations and servers. They are
dependent on tools that can't be installed locally on your computer (e.g. VCS). Thus, you should work on the labs
on one of the lab machines in Hamerschlag Hall 1305 or one of the ECE servers. Please refer the
[ECE IT User Guide](https://userguide.its.cit.cmu.edu/resources/computer-clusters/) for more information about ECE
computer resources.

### Setting Up the 18-447 Tools and Build Environment

All of the 18-447 tools and scripts are located under **/afs/ece.cmu.edu/class/ece447/bin/**. Before you can compile and
run any of your code, you must setup your environment variables. To setup the build environment for the labs, run:

```bash
source /afs/ece.cmu.edu/class/ece447/bin/447setup
```

This sets up your environment variables properly so that you can run the VCS compiler for simulation, the DC compiler
for synthesis, the RISC-V cross-compiler for compiling test programs, and the reference RISC-V simulator. It is
recommended that you put this command in your **.bashrc** file, so that the build environment is setup automatically
every time you log into a machine. To do this, run:

```bash
printf "source /afs/ece.cmu.edu/class/ece447/bin/447setup\n" >> ${HOME}/.bashrc
```

(Please note the use of `>>` in the above to append and not `>`.)

### Creating an AFS Work Directory ###

If you are unfamiliar with using `git`, follow the instructions in Handout #2 to create an AFS work directory.

## Simulating Your Design

### Running Verilog Simulation

To run simulation with your processor design with a specific test, use the command:

```bash
make sim TEST=447inputs/additest.S
```
This will use Synopsys Verilog Compiler Simulator (VCS) to compile your processor into a simulator executable at **outputs/simulation/riscv-sim**, and run simulation with the test program **[447inputs/additest.S](447inputs/additest.S)**
until the simulation finishes. Additionally, the specified test will be assembled. (You can replace `447inputs/additest.S`
in the examples by the path to other tests you want to run. The directory **[447inputs/](447inputs/)** contains
additional testcases.)

### Verifying Your Design

To verify that your design produces the correct results for a given test, use the command:

```bash
make verify TEST=447inputs/additest.S
```

This will take the same steps as the *sim* target. After the test has completed, it will compare the register dump
generated by the testbench to the test's reference register dump (e.g. **[447inputs/additest.reg](447inputs/additest.reg)**),
and notify you if your dump differs from it.

You can also run verification against a batch of tests. For example, to run verification with all tests with a
*.S* extension under the **[447inputs/](447inputs/)** directory, you can run:

```bash
make autograde TESTS=447inputs/*.S
```

In this case, the Makefile only prints out a summary for each test saying whether it passed or failed, and the output of
each individual test is suppressed. The directory **[447inputs/](447inputs/)** may contain more than the required tests for a given lab. 

If you leave the **TESTS** variable unspecified, it defaults to the set of
tests that you are *required* to pass for checkoff for this lab. So, if you want to see if you are ready for checkoff,
run:

```bash
make autograde
```

### Other Makefile Commands

For a complete listing of the Makefile commands and variables, run:

```bash
make help
```
When `make` is misbehaving, it is sometime helpful to run `make veryclean` to clean up the generated temporary files.

## Debugging Your Design

### Test Disassembly Files

When the build system compiles and assembles a test, it also generates a disassembly file for the test. This is useful
when debugging tests as the disassembly shows each instruction along with its address in memory. Naturally, this is
particularity useful when debugging C tests. The disassembly will also show the values of any global variables defined
in the program. The disassembly file will be located at **<test_name>.disassembly.s** (e.g.
**447inputs/additest.disassembly.s**).

### Running the Waveform Viewer

To run the Discovery Visualization Environment (DVE) GUI, or the waveform viewer, you can run:

```bash
make sim-gui TEST=447inputs/additest.S
```

This will take the same steps as the *sim* target, except instead of running the test in simulation to completion, it
will launch the waveform viewer. From here you can run the test in simulation, and observe the signals in your design
over time. The waveform viewer is launched as a background process. Naturally, if you are SSH'ed into a machine, you
must have X11-forwarding enabled to start the DVE (waveform viewer) GUI.

If you are unfamiliar with the waveform viewer, see the Discovery Visualization Environment (DVE) User Guide, which is
located on AFS under
**/afs/ece.cmu.edu/support/synopsys/synopsys.release/vcs-mx_vJ-2014.12/doc/UserGuide/pdf/dve_ug.pdf**.
Chapter 5 is of particular importance, as it goes over how to use the wave window, and chapters 2-4 cover the rest of
the waveform viewer. Some of the waveform viewer's capabilities include searching for the rising edge of signals,
searching for specific values of multi-bit signals, and setting breakpoints in your code, among other things.

### Reference Simulator and Verbose Mode

There is a "golden" reference simulator that you can use to help with debugging. The reference simulator is located at
**/afs/ece.cmu.edu/class/ece447/bin/riscv-ref-sim**. If you have sourced the 18-447 setup script, then you can simply
run:

```bash
riscv-ref-sim 447inputs/additest.S
```
or

```bash
make refsim TEST=447inputs/additest.S
```

While you cannot compare the reference simulator (which is a C simulator) directly with your processor when it's run in
simulation, the reference simulator is still useful for comparing the CPU state of a single-cycle microarchitecture at
specific cycles. The reference simulator can be used to check the register values in your design at a particular cycle.

## Synthesizing Your Design

### Running Synthesis

To synthesize your processor into a physical design, you can run:

```bash
make synth
```

This will use Synopsys Design Compiler (DC) to synthesize your processor into a physical design using a standard library
of ASIC components. The synthesis tool is quite verbose, and so the build system logs all of the tool's output to
**output/synthesis/synthesis.log**. It is **strongly** recommend that you look over the warnings in this log file. If
your critical path looks odd or you're not getting the timing you expected, this log will likely contain the source of
the problem.  A valid implementation in 18-447 must not result in inferred latches.

### Viewing the Timing, Power, and Area Reports

Once synthesis is completed, reports summarizing the timing and critical path, power consumption, and the required area
of your design are generated. To view these reports, you can use the *view-\** targets of the Makefile. For example, to
view the timing report for the design, run:

```bash
make view-timing
```

You can view the power and area reports with the *view-power* and *view-area* targets, respectively. The reports are all
located in **outputs/synthesis**. The timing report is at **outputs/synthesis/timing_riscv_core.rpt**, and the power and
area reports are similarly named. The *view-\** targets will run the *synth* target if any of the reports do not exist.
Otherwise, it will simply display them.

If the critical path of your desgin includes a combinational external memory read, you will see in
**outputs/synthesis/timing_riscv_core.rpt** the critical path arriving at the `riscv_core` module’s address output port
(for instruction or data) and later the corresponding read data input port. In between, there is a *fake memory* delay
path (set to be about 2.4ns) introduced by the synthesis script to account for the combinational external memory read delay.

### Changing the Target Clock Frequency

The synthesis target also supports changing the clock frequency.  Changing the clock frequency forces the synthesis
compiler to work harder to optimize your design in order to meet the timing requirement. This can radically change
the physical design generated by synthesis, affecting the power and area as well. By default, the build system uses
the maximum clock period required for the lab.

The clock period is specified in nanoseconds. For example, to target your processor for synthesis to a 3.0 ns clock,
you can run:

```bash
make synth CLOCK_PERIOD=3.0
```

## Writing Your Own Tests (Same as Lab 1 Part A)

### Writing Tests

Besides the non-comprehensive published tests in **447inputs/**, your simulator will be graded against a set of much
more thorough blind tests.    It is **strongly** recommended that your write your own tests to fully debug your simulator.
The build system supports two types of test programs: assembly and C programs. The only requirements
for both types of test programs is that they must contain a function named `main`. The `main` function is the entry
point for your program, which is where execution will start.

You can create a directory called **private/** for both your assembly and C test programs.  C programs in **private/** are compiled with -O3.

### Generating a Register Dump

When running verification, the build system expects that there is a *.reg* file with the same name as the test (e.g.
**[447inputs/additest.reg](447inputs/additest.reg)**). This is the register dump at the end of the program's execution,
and is used as reference to compare with the implementation's register dump. This register dump can be generated by the
`rdump` command in the C simulator. Thus to generate a register dump for `<test_name>.S` located under the directory
`<path>`, you can use the reference simulator and run:

```bash
printf "go\nrdump <path>/<test_name>.reg\n" | riscv-ref-sim <path>/<test_name>.S
```
You can also use

```bash
make refdump TEST=<path>/<test_name>.S
```

This command will write the register dump to `regdump.reg` at the top-level. Copy `regdump.reg` manually to `<path>/<test_name>.reg` to use it as the reference result for *verify* and *autograde*. It is a good idea to inspect the file's contents as a sanity check first.  

### Writing an Assembly Test

All assembly programs must end with a *.S* extension. Since assembly programs have a *.S* extension, the preprocessor is
run on them before they are assembled. Thus, you have access to preprocessor directives (e.g. `#define`, `#ifndef`,
etc.), and so you can use macros in your program.

The main function must be explicitly declared with the `.text` directive so it ends up the text section, and made
visible to the linker with the `.global` directive. To end a test and terminate simulation, the `ecall` instruction must
be invoked with the value of *0xa* in the *a0* (*x10*) register. Thus, the most bare-bones assembly test program would
look like:

```assembly
    .text
    .global main
main:
    addi x10, x0, 0xa
    ecall
```

You can also add memory to your assembly programs, in the data section. This can accomplished with several different
assembler directives. For zero-initialized memory, the `.space` directive can be used. For initializing memory with
specific values, there are various directives, such as `.word`, `.halfword`, etc. The simulator will only allocate as
much memory as you request, so you must explicitly declare how much memory you need in your program. For example, to
allocate 20 bytes of zero-initialized memory in the data section, you would write:

```assembly
    .data
data_start:
    .space 20
data_end:
```

The RISC-V assembly language also supports pseudoinstructions. These are instructions that the assembler recognizes, but
are not in the RISC-V ISA. When the assembler encounters these instructions, it substitutes one or more actual RISC-V
instructions needed to implement the pseudoinstruction. This can make debugging a bit tricky, since in the assembled
program the pseudoinstructions may be represented by several actual instructions; be sure to use the disassembly of the
program when debugging.

For example, there is the `li` pseudoinstruction, which loads a 32-bit immediate into a register. Depending on the value
of the immediate, this instruction can expand to an `lui` followed by an `ori`. For a complete listing of
pseudoinstructions, see the RISC-V Assembly Programmer's Handbook chapter in the
[RISC-V ISA Specification](https://riscv.org/specifications/).

For an example of a basic assembly test, see **[447inputs/addtest.S](447inputs/addtest.S)**. For an example of an
assembly test that uses memory, see **[447inputs/memtest0.S](447inputs/memtest0.S)**. For a complete listing and
description of supported assembler directives, see the
[GNU Assembler (GAS) Guide](https://sourceware.org/binutils/docs/as/Pseudo-Ops.html#Pseudo-Ops).

### Writing a C Test

All C programs must end with a *.c* extension. The only requirement for a C program is that it contains a function named
`main`. The C program is wrapped by a small assembly startup function, located in the file
**[447runtime/crt0.S](447runtime/crt0.S)** (CRT stands for C runtime). This function sets up the environment for the C
program to run, invokes the user program, and, when main returns, invokes the `ecall` instruction as described before to
terminate simulation.

The RISC-V ABI permits doubleword return values. Thus, the lower 32-bits of the user program's return value are placed
in the *x2* (*sp*) register, while the upper 32-bits are placed in the *x3* (*gp*) register.

The C program test runs in a very minimal environment. None of the C standard library functions are available, and no
external functions can be called from the program, other than compiler intrinsic functions. The only functions that can
be called are ones which are defined in the same file. Other than that, all parts of the C language are supported.

The Makefile compiles programs in **benchmarks/** and **private/** with -O by default.
The Makefile compiles programs in **benchmarksO3/** and **privateO3/** with -O3.

The Makefile is configured to compile for the RV32IM target (`-march=rv32im`) .  Although you are implmenting
only the MUL instruction from the M extension, it is sufficient for the testcases we provide.

By setting the compile target to RV32I in the Makefile (`-march=rv32i`), 
you can, in general, use integer multiplication and even floating-point from within C programs, 
even if the processor only supports the RV32I subset of the RISC-V ISA.
In this case, the code is linked against the GCC library, which provides
software implementations of multiplications and 
floating point operations with only RV32I instructions.

For an example of a C test, see **[benchmarks/mmmFP.c](benchmarks/mmmFP.c)**. Please note this C program 
requires floating point operations which is not supported by RV32IM. 
When GCC is set to `-march=rv32i` or `-march=rv32im`, it will include *libgcc* functions to emulate the needed RISC-V instructions from the F extension.

Try building and running **[benchmarks/mmmFP.c](benchmarks/mmmFP.c)**.

```bash
make run TEST=benchmarks/mmmFP.c
```
Afterwards, take a look at the resulting **benchmarks/mmmFP.disassembly.s** for calls to the emulation functions, `__mulsf3` and `__addsf3`.

## Appendix

### Overview of the Build System

The build system utilizes the RISC-V GCC toolchain to compile and assemble test programs. When a test is run for
simulation, it is first compiled into an ELF executable, **<test_name>.elf**, using RISC-V GCC. A linker script, located
at **[447runtime/test_program.ld](447runtime/test_program.ld)**, is used to map all program memory into 4 distinct
sections. There are two text sections, one for user code (*.text*) and one for kernel code (*.ktext*). The text sections
contain the corresponding code and any read-only global variables. In addition, there are two data sections, one for
user data (*.data*) and one for kernel data (*.kdata*). The data sections contain the corresponding writable global
variables, and any uninitialized global variables (from the *.bss* section).

The simulator expects the 4 sections to be in binary format, in separate files, as parsing an ELF binary is a bit
complex. Thus, the build system utilizes `objcopy` to extract the 4 sections from the ELF binary, placing them in the
corresponding **<test_name>.<section_name>.bin** files. It also concatenates the user and kernel *.bss* sections to the
end of the corresponding *.data* sections into one binary file. A disassembly file for the test also generated from the
ELF executable, under **<test_name>.disassembly.s**.

The build system then compiles the simulator. To reduce filesystem clutter, the simulator is built in
**outputs/simulation**. To simplify the code base, the section binary data files for the test being run are copied to
the output directory. When the simulator starts, it loads the data for each memory segment from the corresponding binary
files for each program section. In addition, a stack segment, shared between the kernel and user code, is setup by the
simulator. The simulator also initializes the *sp* (*x2*) register to point to the end of the stack, and the *gp* (*x3*)
register to point to the beginning of the user data segment. Naturally, the *pc* register is then initialized to point
at the beginning of the user text segment, and the program begins execution.

For verification, the build system assumes that there is a register dump under **<test_name>.reg** that has the expected
register state when the program finishes execution. The simulator generates a register dump when the program finishes,
and the build system uses `sdiff` to determine if the two register dumps match.

For synthesis, as with simulation, it is done in its own directory, **output/synthesis**, to reduce filesystem clutter.
A Design Compiler (DC) script is used to control how the synthesis tools synthesizes the design. This script is located
at **[dc/dc_synth.tcl](dc/dc_synth.tcl)**. The synthesis script uses maximum timing effort for synthesizing the design,
but it does not utilize the `compile_ultra` command. Additionally, the script is setup to reduce how often the DC
synthesis tool crashes, as it is often prone to do with designs that cannot meet timing.

### Useful Links

[RISC-V ISA Specification](https://riscv.org/specifications/) - [https://riscv.org/specifications/](https://riscv.org/specifications/)

[IEEE 1800-2012 SystemVerilog Language Reference Manual (LRM)](http://www.ece.uah.edu/~gaede/cpe526/2012%20System%20Verilog%20Language%20Reference%20Manual.pdf) - [http://www.ece.uah.edu/~gaede/cpe526/2012%20System%20Verilog%20Language%20Reference%20Manual.pdf](http://www.ece.uah.edu/~gaede/cpe526/2012%20System%20Verilog%20Language%20Reference%20Manual.pdf)

[ECE IT User Guide](https://userguide.its.cit.cmu.edu/resources/computer-clusters/) - [https://userguide.its.cit.cmu.edu/resources/computer-clusters/](https://userguide.its.cit.cmu.edu/resources/computer-clusters/)

Discovery Visualization Environment (DVE) User Guide - /afs/ece.cmu.edu/support/synopsys/synopsys.release/vcs-mx_vJ-2014.12/doc/UserGuide/pdf/dve_ug.pdf

[GNU Assembler (GAS) Guide](https://sourceware.org/binutils/docs/as/Pseudo-Ops.html#Pseudo-Ops) - [https://sourceware.org/binutils/docs/as/Pseudo-Ops.html#Pseudo-Ops](https://sourceware.org/binutils/docs/as/Pseudo-Ops.html#Pseudo-Ops)

### Special Instructions for Lab 2

Please refer to course handout for current-year project requirements and logistics.

### Special Instructions for Lab 3

Please refer to course handout for current-year project requirements and logistics.

### Special Instructions for Lab 4

Please refer to course handout for current-year project requirements and logistics.
