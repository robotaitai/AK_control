import motorsParams
import time
def float_to_uint(x, x_min, x_max, numBits):
    span = x_max - x_min
    offset = x_min
    # Attempt to speedup by using pre-computation. Not used currently.
    if numBits == 16:
        bitRange = motorsParams.maxRawPosition
    elif numBits == 12:
        bitRange = motorsParams.maxRawVelocity
    else:
        bitRange = 2 ** numBits - 1
    return int(((x - offset) * (bitRange)) / span)


def uint_to_float(x_int, x_min, x_max, numBits):
    span = x_max - x_min
    offset = x_min
    if numBits == 16:
        bitRange = motorsParams.maxRawPosition
    elif numBits == 12:
        bitRange = motorsParams.maxRawVelocity
    else:
        bitRange = 2 ** numBits - 1
    return ((x_int * span) / (bitRange)) + offset


def waitOhneSleep(dt):
    startTime = time.time()
    while time.time() - startTime < dt:
        pass

