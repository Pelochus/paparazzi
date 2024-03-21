# Connecting and setting up the debugger
target extended-remote localhost:3333
layout regs

# OpenOCD config
set mem inaccessible-by-default off
enable breakpoints
break __cpu_init

# Reset the target
monitor reset halt
monitor halt
