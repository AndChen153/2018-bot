from magicbot import tunable
from wpilib import PIDController
from robotpy_ext.common_drivers import navx

from components.drivetrain import Drivetrain
from .base_pid_controller import BasePIDComponent
from .angle_controller import AngleController


class PositionController(BasePIDComponent):

    drivetrain = Drivetrain

    kP = tunable(0.1)
    kI = tunable(0.0)
    kD = tunable(0.0)
    kF = tunable(0.0)
    kToleranceInches = tunable(2)
    kIzone = tunable(0.25)

    angle_controller = AngleController

    # kAngleP = tunable(0.1)
    # kAngleI = tunable(0.0)
    # kAngleD = tunable(0.0)
    # kAngleF = tunable(0.0)
    kAngleP = 0.1
    kAngleI = 0
    kAngleD = 0
    kAngleF = 0
    kAngleMax = 0.3

    # Angle correction factor
    angle = 0

    def __init__(self):
        super().__init__(self.get_position, 'position_controller')
        self.set_abs_output_range(0.16, 0.8)

        # Angle correction PID controller - used to maintain a straight
        # heading while the encoders track distance traveled.
        self.angle_pid_controller = PIDController(
            Kp=self.kAngleP, Ki=self.kAngleI, Kd=self.kAngleD, Kf=self.kAngleF,
            source=self.angle_controller.get_angle,
            output=self.pidWriteAngle)
        self.angle_pid_controller.setInputRange(-180, 180)
        self.angle_pid_controller.setContinuous(True)
        self.angle_pid_controller.setOutputRange(-self.kAngleMax,
                                                 self.kAngleMax)

    def get_position(self):
        return self.drivetrain.get_position()

    def reset_position_and_heading(self):
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
                self.drivetrain.differential_drive(self.rate, self.angle)

    def stop(self):
        self.drivetrain.differential_drive(0)
        self.angle_pid_controller.disable()
