from collections import namedtuple
from networktables import NetworkTables

from common.util import scale

TargetingData = namedtuple('TargetingOffset',
                           ['found', 'x', 'y', 'area', 'skew'])

LED_MODE_ON = 0
LED_MODE_OFF = 1
LED_MODE_BLINK = 2

CAMERA_MODE_VISION = 0
CAMERA_MODE_DRIVER = 1


class Targeting:
    def setup(self):
        self.nt = NetworkTables.getTable('limelight')

    def set_led_mode(self, mode):
        self.nt.setNumber('ledMode', mode)

    def set_camera_mode(self, mode):
        self.nt.setNumber('camMode', mode)

    def get_data(self):
        return TargetingData(found=self.nt.getBoolean('tv', False),
                             x=scale(self.nt.getNumber('tx', 0),
                                     -27, 27, -1, 1),
                             y=scale(self.nt.getNumber('ty', 0),
                                     -20.5, 20.5, 1, 1),
                             area=scale(self.nt.getNumber('ta', 0),
                                        0, 100, 0, 1),
                             skew=scale(self.nt.getNumber('ts', 0),
                                        -90, 0, 0, 1))

    def execute(self):
        pass
