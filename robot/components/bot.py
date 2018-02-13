from enum import Enum
from wpilib import Spark
# from networktables import NetworkTables


class LedState(Enum):
    RAINBOW = -0.89
    BREATH = 0.11


class Bot:

    led_driver = Spark

    def setup(self):
        self.led_state = LedState.RAINBOW

    def set_led_state(self, state):
        self.led_driver.set(state)

    def execute(self):
        self.led_driver.set(self.state)
