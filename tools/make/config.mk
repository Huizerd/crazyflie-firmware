## Automatically enter bootloader mode before flashing
CLOAD_CMDS = -w radio://0/80/2M

## LPS settings ----------------------------------------------------
## Set operation mode of the LPS system (auto is default)
## TWR
# CFLAGS += -DLPS_TWR_ENABLE=1
## TDoA2
# LPS_TDOA_ENABLE=1
## TDoA3
# LPS_TDOA3_ENABLE=1

## TDoA 3 - experimental
# Enable 2D positioning. The value (1.2) is the height that the tag will move at. Only use in TDoA 3
# CFLAGS += -DLPS_2D_POSITION_HEIGHT=1.2

## Enable longer range (lower bit rate). Only use in TDoA 3
# Note: Anchors must also be built with this flag
# CFLAGS += -DLPS_LONGER_RANGE

## Full LPS TX power.
# CFLAGS += -DLPS_FULL_TX_POWER
