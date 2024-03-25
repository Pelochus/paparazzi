# Hey Emacs, this is a -*- makefile -*-
#
# crazyflie_2.1_bare.makefile
#
#

BOARD=crazyflie
BOARD_VERSION=2.1
BOARD_DIR=$(BOARD)
BOARD_CFG=\"boards/$(BOARD_DIR)/$(BOARD).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/apogee.ld

HARD_FLOAT=yes

# -----------------------------------------------------------------------

# default flash mode is via usb dfu bootloader (luftboot)
# other possibilities: DFU-UTIL, SWD, JTAG_BMP, STLINK, SERIAL
FLASH_MODE ?= DFU-UTIL

# while the crazyflie doesn't have luftboot, it has a bootloader until same offset
# see also https://wiki.bitcraze.io/projects:crazyflie2:development:dfu
HAS_LUFTBOOT ?= 1
ifeq (,$(findstring $(HAS_LUFTBOOT),0 FALSE))
$(TARGET).CFLAGS+=-DLUFTBOOT
$(TARGET).LDFLAGS+=-Wl,-Ttext=0x8004000
DFU_ADDR = 0x8004000
endif

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= 3
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 2
GPS_LED            ?= 4
SYS_TIME_LED       ?= 1
CHARGING_LED       ?= 5

#
# default uart configuration
#

# Over NRF/Syslink
MODEM_PORT ?= syslink

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm

