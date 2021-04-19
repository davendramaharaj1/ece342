# ------------------------------------------------------------------------------
# Top Level Simulation Script to source msim_setup.tcl
# ------------------------------------------------------------------------------
set QSYS_SIMDIR obj/default/runtime/sim
source msim_setup.tcl
# Copy generated memory initialization hex and dat file(s) to current directory
file copy -force C:/ECE342/lab4/part1/software/leds_sw/mem_init/hdl_sim/nios_system_onchip_memory2_0.dat ./ 
file copy -force C:/ECE342/lab4/part1/software/leds_sw/mem_init/nios_system_onchip_memory2_0.hex ./ 
