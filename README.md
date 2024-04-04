# Main README
If you want to read the original README, please refer to this [link](https://github.com/paparazzi/paparazzi/)

# About
This repo is the code for my bachelor's thesis, which main repo is [here](https://github.com/Pelochus/bt-crazyflies/).
There is lot of extra info there, in fact, you should start over there in order to better understand this repo.
There is also some useful scripts that install dependencies needed for the additions in this fork.

# What's included in this fork
The following sections are what is included in this fork that is (currently) not included in the main repo of Paparazzi

## Debugging
### OpenOCD for Crazyflie 2.1
**Remember to set ```RTOS_DEBUG``` to 1 in ```conf/airframes/UGR/crazyflie_2.1.xml``` for debugging ChibiOS**.
Added a Makefile for easy debugging with OpenOCD. This Makefile assumes you have Bash, Konsole and other dependencies installed.
Run:

```make -f Makefile.debug```

This will launch an OpenOCD server and two new terminals, one with telnet and the other one with GDB.
It also dumps the firmware in ARM assembly with objdump to the folder objdump.

It is also very recommended (easier and works better) to follow this
[guide](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/openocd_gdb_debugging/)
from Bitcraze (the official debugging guide for Crazyflie).
Use the updated file ```.vscode/launch.json``` from this repo.

### Python with VSCode
In the file ```.vscode/launch.json``` from this repo, there is also configuration for debugging the current Python file.
This was used for debugging the ```crazyradio2ivy.py``` file

### Debug Setup photos
![DebugSetup](https://github.com/Pelochus/paparazzi/blob/master/img/DebugSetup.jpg)
![DebugSetup2](https://github.com/Pelochus/paparazzi/blob/master/img/DebugSetup2.jpg)

## Extra modules and config for Crazyflie 2.1
### GVF for rotorcrafts

### Relative positioning with Loco System (Crazyflie)

### Other
- Added initial bare-metal implementation for Crazyflie 2.1
- Added firmware configuration for Crazyflie in `conf/airframes/UGR/`
- Added new flightplan for Crazyflie in `conf/flight_plans/UGR/`
- Added new telemetry option for Crazyflie GVF in `conf/telemetry/GVF`
- Added Crazyflie 2.1 GVF in `conf/conf_ugr.xml`
- Addec Control Panel for Crazyflie in `conf/userconf/UGR/`
- blah blah

### Useful links, docs and related to this repo
- https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/openocd_gdb_debugging/
- https://openocd.org/pages/documentation.html
