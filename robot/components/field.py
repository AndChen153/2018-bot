from enum import IntEnum
from wpilib import DriverStation
# from networktables import NetworkTables


class SwitchState(IntEnum):
    LEFT = 0
    RIGHT = 1
    UNKNOWN = 2


class Field:

    def get_switch_side():
        message = DriverStation.getInstance().getGameSpecificMessage().lower()
        if message:
            if message[0] == 'l':
                return SwitchState.LEFT
            elif message[0] == 'r':
                return SwitchState.RIGHT
        else:
            return SwitchState.UNKNOWN

    def execute(self):
        # NetworkTables.putValue('/field/switch_side', self.get_switch_side())
        pass
