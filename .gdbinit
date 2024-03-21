target extended-remote localhost:3333
layout regs
break __cpu_init
monitor reset
monitor halt
