import math

maxRawPosition = 2 ** 16 - 1  # 16-Bits for Raw Position Values
maxRawVelocity = 2 ** 12 - 1  # 12-Bits for Raw Velocity Values
# CAN frame packing/unpacking (see `struct can_frame` in <linux/can.h>)
# 8 bytes of data is sent to the motor
can_frame_fmt_send = "=IB3x8s"
# 6 bytes are received from the motor
can_frame_fmt_recv = "=IB3x6s"

# List of Motors Supported by this Driver
legitimate_motors = [
    "AK80_6_V1",
    "AK80_6_V1p1",
    "AK80_6_V2",
    "AK80_9_V1p1",
    "AK80_9_V2",
    "AK10_9_V2",
    'AK80_64_V2'
]

# Constants for conversion
# Working parameters for AK80-6 V1.0 firmware
AK80_6_V1_PARAMS = {
    "P_MIN": -95.5,
    "P_MAX": 95.5,
    "V_MIN": -45.0,
    "V_MAX": 45.0,
    "KP_MIN": 0.0,
    "KP_MAX": 500,
    "KD_MIN": 0.0,
    "KD_MAX": 5.0,
    "T_MIN": -18.0,
    "T_MAX": 18.0,
    "AXIS_DIRECTION": -1
}

# Working parameters for AK80-6 V1.1 firmware
AK80_6_V1p1_PARAMS = {
    "P_MIN": -12.5,
    "P_MAX": 12.5,
    "V_MIN": -22.5,
    "V_MAX": 22.5,
    "KP_MIN": 0.0,
    "KP_MAX": 500,
    "KD_MIN": 0.0,
    "KD_MAX": 5.0,
    "T_MIN": -12.0,
    "T_MAX": 12.0,
    "AXIS_DIRECTION": -1
}

# Working parameters for AK80-6 V2.0 firmware
AK80_6_V2_PARAMS = {
    "P_MIN": -12.5,
    "P_MAX": 12.5,
    "V_MIN": -38.2,
    "V_MAX": 38.2,
    "KP_MIN": 0.0,
    "KP_MAX": 500.0,
    "KD_MIN": 0.0,
    "KD_MAX": 5.0,
    "T_MIN": -12.0,
    "T_MAX": 12.0,
    "AXIS_DIRECTION": 1
}

# Working parameters for AK80-9 V1.1 firmware
AK80_9_V1p1_PARAMS = {
    "P_MIN": -12.5,
    "P_MAX": 12.5,
    "V_MIN": -22.5,
    "V_MAX": 22.5,
    "KP_MIN": 0.0,
    "KP_MAX": 500,
    "KD_MIN": 0.0,
    "KD_MAX": 5.0,
    "T_MIN": -18.0,
    "T_MAX": 18.0,
    "AXIS_DIRECTION": 1
}

# Working parameters for AK80-9 V2.0 firmware
AK80_9_V2_PARAMS = {
    "P_MIN": -12.5/4,
    "P_MAX": 12.5/4,
    "V_MIN": -25.64,
    "V_MAX": 25.64,
    "KP_MIN": 0.0,
    "KP_MAX": 500.0,
    "KD_MIN": 0.0,
    "KD_MAX": 5.0,
    "T_MIN": -18.0,
    "T_MAX": 18.0,
    "AXIS_DIRECTION": 1,
    "GEAR_RATIO": 9
}

# Working parameters for AK80-64 V2.0 firmware
AK80_64_V2_PARAMS = {
    "P_MIN": -12.5,
    "P_MAX": 12.5,
    "V_MIN": -25.64,
    "V_MAX": 25.64,
    "KP_MIN": 0.0,
    "KP_MAX": 500.0,
    "KD_MIN": 0.0,
    "KD_MAX": 5.0,
    "T_MIN": -144.0,
    "T_MAX": 144.0,
    "AXIS_DIRECTION": 1,
    "RATIO": 64
}
# Working parameters for AK80-64 V2.0 firmware
AK10_9_V2_PARAMS = {
    "P_MIN": -180.0,
    "P_MAX": 180.0,
    "V_MIN": -25.64,
    "V_MAX": 25.64,
    "KP_MIN": 0.0,
    "KP_MAX": 500.0,
    "KD_MIN": 0.0,
    "KD_MAX": 5.0,
    "T_MIN": -18.0,
    "T_MAX": 18.0,
    "AXIS_DIRECTION": 1,
    "RATIO": 9
}