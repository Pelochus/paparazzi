# Main README
If you want to read the original README, please refer to this [link](https://github.com/paparazzi/paparazzi/)

# About
This repo is the code for my bachelor's thesis, which main repo is [here](https://github.com/Pelochus/bt-crazyflies/).
There is lot of extra info there, in fact, you should start over there in order to better understand this repo.
There is also some useful scripts that install dependencies needed for the additions in this fork.

# What's included in this fork
The following sections are what is included in this fork that is (currently) not included in the main repo of Paparazzi

## Debugging with OpenOCD for Crazyflie 2.1
Added a Makefile for easy debugging with OpenOCD. This Makefile assumes you have Bash, Konsole and other dependencies installed.
Run:

```make -f Makefile.debug```

This will launch an OpenOCD server and two new terminals, one with telnet and the other one with GDB.
It also dumps the firmware in ARM assembly with objdump to the folder objdump.

# TODO INCLUDE DEBUGGING SETUP PHOTO HERE, COULD BE USEFUL

### Useful links and docs
https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/openocd_gdb_debugging/
https://openocd.org/pages/documentation.html

## Extra modules and config for Crazyflie 2.1
Added firmware configuration XML in TODO
Added module blah blah TODO
