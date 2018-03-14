from enum import IntEnum
import wpilib
from networktables import NetworkTables
from controllers.angle_controller import AngleController


class SwitchState(IntEnum):
    LEFT = 0
    RIGHT = 1
    UNKNOWN = 2


class Field:

    angle_controller = AngleController

    def get_switch_side(self):
        message = wpilib.DriverStation.getInstance().getGameSpecificMessage() \
                        .lower()
        if message:
            if message[0] == 'l':
                return SwitchState.LEFT
            elif message[0] == 'r':
                return SwitchState.RIGHT
        else:
            return SwitchState.UNKNOWN

    def execute(self):
        robot_table = NetworkTables.getTable('robot')
        robot_table.putValue('switch_side', self.get_switch_side())
        # robot_table.putValue('angle', self.angle_controller.get_angle())
        robot_table.putValue('time', wpilib.Timer.getMatchTime())
