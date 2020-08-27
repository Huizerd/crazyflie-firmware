PLATFORM_HELP_cf2 = Crazyflie2 used as sensor suite
PLATFORM_NAME_cf2 = CF-board platform

CPU=stm32f4

######### Sensors configuration ##########
CFLAGS += -DSENSOR_INCLUDED_BMI088_BMP388
PROJ_OBJ += sensors_bmi088_bmp388.o

CFLAGS += -DSENSOR_INCLUDED_MPU9250_LPS25H
PROJ_OBJ += sensors_mpu9250_lps25h.o

CFLAGS += -DSENSOR_INCLUDED_BMI088_SPI_BMP388
PROJ_OBJ += sensors_bmi088_spi_bmp388.o

######### Stabilizer configuration ##########
ESTIMATOR          ?= any
CONTROLLER         ?= PID  # force PID controller for obtaining velocity commands
POWER_DISTRIBUTION ?= stock

######### Force Mavic driver ##########
CFLAGS += -DDECK_FORCE=mavic

######### Communications configuration ##########
# CFLAGS += -DENABLE_UART2
