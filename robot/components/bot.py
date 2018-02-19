from enum import Enum
from wpilib import Spark


class LedState(float, Enum):
    RAINBOW = -0.89
    BREATH = 0.11
    GREEN = 0.77
    RED = 0.61


class Bot:

    led_driver = Spark

    def setup(self):
        self.led_state = LedState.RAINBOW

    def set_led_state(self, state):
        self.led_state = state

    def execute(self):
        self.led_driver.set(self.led_state)
