# dc_synth.tcl
#
# RISC-V 32-bit Processor
#
# ECE 18-447
# Carnegie Mellon University
#
# This file is the 18-447 Synopsys DC script.
#
# The DC script handles synthesizing the processor into a design. This file sets
# the settings and options for the DC tools to properly synthesize the
# processor, synthesizes the design, and then saves the timing, power, and area
# synthesis results to file.
#
# This script was adapted from the 18-341 Synopsys DC script for project 4.
#
# Authors:
#   - 2016 - 2017: Brandon Perez

#-------------------------------------------------------------------------------
# Command Line Arguments and Sanity Checks
#-------------------------------------------------------------------------------

# This script takes the top-level project directory and the clock period for the
# design as arguments.
set usage "Usage: dc_shell-xg-t -f dc/dc_synth.tcl -x 'set project_dir <path>; "
append usage "set lab_18447 <1b|2|3|4a|4b>; \[set clock_period <period>\]'"
if {![info exists project_dir]} {
    puts "Error: Project directory not specified."
    puts $usage
    exit 1
} elseif {![info exists lab_18447]} {
    puts "Error: Lab number not specified."
    puts $usage
    exit 1
}

# Convert the project directory to an absolute path, and set the paths for the
# source file directories
set project_dir [exec readlink -m $project_dir]
set 447_src_dir $project_dir/447src
set 447_include_dir $project_dir/447include
set student_src_dir $project_dir/src

# Check that the project and source file directories exist
if {![file exists $project_dir]} {
    puts "Error: $project_dir: Project directory does not exist."
    exit 1
} elseif {![file exists $447_src_dir]} {
    puts -nonewline "Error: $project_dir: Project directory does not have a "
    puts "447src directory."
    exit 1
} elseif {![file exists $447_include_dir]} {
    puts -nonewline "Error: $project_dir: Project directory does not have a "
    puts "447include directory."
    exit 1
} elseif {![file exists $student_src_dir]} {
    puts "Error: $project_dir: Project directory does not have src directory."
    exit 1
}

# If the user didn't specify a clock period, then select the appropriate one
# based on the lab number. Otherwise, check that the clock period is valid.
set valid_labs "{1b, 2, 3, 4a, 4b}"
if {![info exists clock_period]} {
    switch -- $lab_18447 {
        "1b" - "4a" - "4b" {
            set clock_period 10
        }
        "2" - "3" {
            set clock_period 4.5
        }
        default {
            puts -nonewline "Error: Invalid lab '$lab_18447' specified. Lab "
            puts "number must be one of $valid_labs."
            exit 1
        }
    }
} elseif {![string is double $clock_period] || $clock_period <= 0} {
    puts -nonewline "Error: Clock period '$clock_period' is not a positive "
    puts "double value."
    exit 1
}

#-------------------------------------------------------------------------------
# Setup and Variables
#-------------------------------------------------------------------------------

# Set the source files and include directories for the project, making sure that
# they are absolute paths.
set 447_src [exec find -L $447_src_dir -type f -name "*.v" -o -name "*.sv" | \
        sort]
set student_src [exec find -L $student_src_dir -type f -name "*.v" \
        -o -name "*.sv" | sort]
set student_src_subdirs [exec find -L $student_src_dir -type d | sort]
set src [concat $447_src $student_src]

# Set the libraries used to implement the design
set target_library /afs/ece/class/ece447/tools/synopsys/typical.db
set link_library /afs/ece/class/ece447/tools/synopsys/typical.db

# Set top module and add the 447src directory and student subdirectories to the
# search path for includes.
set top_module riscv_core_timing
set search_path [concat $search_path $447_include_dir $student_src_subdirs]

# Setup the compiler for parallel compilation with the max number of threads.
set cores [exec getconf _NPROCESSORS_ONLN]
set threads [expr min($cores, 16)]
set_host_options -max_cores $threads

# Define a library where our synthesized design files will be stored
define_design_lib WORK -path "./work"

#-------------------------------------------------------------------------------
# Design Synthesis
#-------------------------------------------------------------------------------

# Syntax check the source files, and create library objects for the files for
# design synthesis.
if {![analyze -format sverilog -lib WORK -define LAB_18447="$lab_18447" \
        $src]} {
    exit 1
}

# Synthesize the design into a technology-independent design, and link the
# design to library components and references to other modules in the design.
if {![elaborate $top_module -lib WORK]} {
    exit 1
}

# Even though the elaborate command already performs linking, it succeeds even
# if linking fails, so link is run again to check for this case.
if {![link]} {
    exit 1
}

#-------------------------------------------------------------------------------
# Design and Optimization Constraints
#-------------------------------------------------------------------------------

# Set the design to optimize as the top module. All modules in the hierarchy
# below will also be optimized.
current_design $top_module

# Create a clock for the design, and set its period. This can be changed to a
# lower value to force the synthesis tool to work harder to optimize the design.
create_clock -period $clock_period clk

# Model a semi-realistic delay for main memory by setting a delay on the input
# ports to the top module.
set real_inputs [remove_from_collection [all_inputs] clk]
set_input_delay -clock clk 0.0 $real_inputs
set_output_delay -clock clk 0.0 [all_outputs]

# Set the maximum allowed combinational delay to be the clock period.
set_max_delay $clock_period [all_outputs]

# For some reason, DC very heavily prefers using ripple-carry adders when
# implementing addition, even if the design isn't meeting timing. Thus, we force
# the compiler not to use them, so it will instead use carry-lookahead adders.
set_dont_use standard.sldb/DW01_addsub/rpl
set_dont_use standard.sldb/DW01_add/rpl
set_dont_use standard.sldb/DW01_sub/rpl

#-------------------------------------------------------------------------------
# Design Optimization
#-------------------------------------------------------------------------------

# Optimize the design. The effort options tell the compiler to fully optimize
# for area and timing. Boundary optimization allows the compiler to optimize
# between module boundaries. Incremental mapping makes the compiler
# incrementally improve the design by experimenting with different approaches.
# This also prevents the DC compiler from crashing, which it is prone to do if
# the timing constraints for the design are not met.
if {![compile -map_effort high -area_effort high -boundary_optimization \
        -incremental_mapping]} {
    exit 1
}

#-------------------------------------------------------------------------------
# Report Generation
#-------------------------------------------------------------------------------

# Check the final optimized design for any inconsistencies and report them.
if {![check_design]} {
    exit 1
}

# Report the area, timing, and power consumption of the design.
report_area > area_riscv_core.rpt
report_timing > timing_riscv_core.rpt
report_power -hierarchy > power_riscv_core.rpt

# Output the netlist of the final synthesized design in verilog format.
write -format verilog -output netlist_riscv_core.sv
exit 0
