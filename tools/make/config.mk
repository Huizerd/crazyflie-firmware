## Automatically enter bootloader mode before flashing
# Cf 2.0
# CLOAD_CMDS = -w radio://0/100/2M
# Cf 2.1
CLOAD_CMDS = -w radio://0/80/2M

## Set the positioning system in TWR mode on startup
# LPS_TDOA_ENABLE=1

## Set the positioning system in TDoA2 mode on startup
# LPS_TDOA_ENABLE=1

## Set the positioning system in TDoA3 mode on startup
# LPS_TDOA3_ENABLE=1

## TDoA 3 - experimental -------------------------------------------
# Enable 2D positioning. The value (1.2) is the height that the tag will move at
# Only use in TDoA 3
# CFLAGS += -DLPS_2D_POSITION_HEIGHT=1.2

# Enable longer range (lower bit rate)
# Only use in TDoA 3
# Note: Anchors must also be built with this flag
CFLAGS += -DLPS_LONGER_RANGE
