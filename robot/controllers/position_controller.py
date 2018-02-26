from magicbot import tunable
from wpilib import PIDController
import hal

from components.drivetrain import Drivetrain
from .base_pid_controller import BasePIDComponent
from .angle_controller import AngleController


class PositionController(BasePIDComponent):

    drivetrain = Drivetrain

    kP = tunable(0.01)
    kI = tunable(0)
    kD = tunable(0.005)
    kF = tunable(0.0)
    kToleranceInches = tunable(0.2 if hal.HALIsSimulation() else 0.75)
    kIzone = tunable(0)

    angle_controller = AngleController

    kAngleP = 0.05 if hal.HALIsSimulation() else 0.1
    kAngleI = 0
    kAngleD = 0
    kAngleF = 0
    kAngleMax = 0.18

    # Angle correction factor
    angle = 0

    def __init__(self):
        super().__init__(self.get_position, 'position_controller')
        self.set_abs_output_range(0.14, 0.7)

        # Angle correction PID controller - used to maintain a straight
        # heading while the encoders track distance traveled.
        self.angle_pid_controller = PIDController(
            Kp=self.kAngleP, Ki=self.kAngleI, Kd=self.kAngleD, Kf=self.kAngleF,
            source=self.get_angle,
            output=self.pidWriteAngle)
        self.angle_pid_controller.setInputRange(-180, 180)
        self.angle_pid_controller.setContinuous(True)
        self.angle_pid_controller.setOutputRange(-self.kAngleMax,
                                                 self.kAngleMax)

    def get_position(self):
        return self.drivetrain.get_position()

    def get_angle(self):
        return self.angle_controller.get_angle()

    def reset_position_and_heading(self):
        self.drivetrain.shift_low_gear()
        self.drivetrain.reset_position()
        self.angle_controller.reset_angle()
        self.angle_pid_controller.setSetpoint(0)

    def move_to(self, position):
        self.setpoint = position
        self.angle_pid_controller.enable()

    def is_at_location(self):
        return self.enabled and \
            abs(self.get_position() - self.setpoint) < self.kToleranceInches

    def pidWrite(self, output):
        self.rate = -output

    def pidWriteAngle(self, angle):
        self.angle = angle

    def execute(self):
        super().execute()

        if self.rate is not None:
            if self.is_at_location():
                self.stop()
            else:
                self.drivetrain.differential_drive(self.rate, self.angle,
                                                   squared=False, force=True)

    def stop(self):
        self.drivetrain.differential_drive(0)
        self.angle_pid_controller.disable()

    def on_disable(self):
        self.stop()
