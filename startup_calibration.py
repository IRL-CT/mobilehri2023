import odrive
from odrive.enums import *

if __name__ == '__main__':
    odrv = odrive.find_any()
    odrv.clear_errors()
    axis0 = odrv.axis0
    axis1 = odrv.axis1
    axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    axis1.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE