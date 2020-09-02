# Make configuration for the UWB Board platform

PLATFORM_HELP_uwb = UWB Board platform, independent board for uwb navigation
PLATFORM_NAME_uwb = UWB Board

CPU=stm32f4

# Force the device string until we can get it from the board itself
CFLAGS += -DDEVICE_TYPE_STRING_FORCE="UB10"
# Force start until we have a better solution
CFLAGS += -DFORCE_START

######### Sensors configuration ##########
CFLAGS += -DDWM_FORCE_TX_POWER=0x1F1F1F1Ful
CFLAGS += -DSENSOR_INCLUDED_UWB
PROJ_OBJ += sensors_uwb.o

######### Stabilizer configuration ##########
CFLAGS += -DNO_IMU
ESTIMATOR          ?= mhe
CONTROLLER         ?= Any # one of Any, PID, Mellinger, INDI
POWER_DISTRIBUTION ?= stock

######### Communication configuration ##########
#PROJ_OBJ += uart6.o