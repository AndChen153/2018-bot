from magicbot import tunable
from robotpy_ext.common_drivers import navx

from components.drivetrain import Drivetrain
from .base_pid_controller import BasePIDComponent


class AngleController(BasePIDComponent):
    """
    When enabled, controls the angle of the robot.
    """

    drivetrain = Drivetrain

    # kP = tunable(0.002)
    # kI = tunable(0.00002)
    # kD = tunable(0.0005)
    # kF = tunable(0.0)

    kP = tunable(0.01)
    kI = tunable(0.0006)
    kD = tunable(0.02)
    kF = tunable(0.005)
    kToleranceDegrees = tunable(0.1)
    kIzone = tunable(0.25)

    navx = navx.AHRS

    def __init__(self):
        super().__init__(self.get_angle, 'angle_controller')

        self.set_abs_output_range(0.1, 0.4)

        if hasattr(self, 'pid'):
            self.pid.setInputRange(-180.0, 180.0)
            self.pid.setOutputRange(-1.0, 1.0)
            self.pid.setContinuous(True)

        self.report = 0

    def get_angle(self):
        """
        Return the robot's current heading.
        """
        print('current_angle', self.navx.getYaw())
        return self.navx.getYaw()

    def align_to(self, angle):
        """
        Move the robot and turns it to a specified absolute direction.
        :param angle: Angle value from 0 to 359
        """
        self.setpoint = angle

    def is_aligned(self):
        """
        Return True if robot is pointing at specified angle.
        Note: Always returns False when move_at_angle is not being called.
        """
        if not self.enabled:
            return False

        return self.is_aligned_to(self.setpoint)

    def is_aligned_to(self, setpoint):
        """
        :param setpoint: Setpoint value to check alignment against
        :return: True if aligned False if not
        """
        angle = self.get_angle()

        # compensate for wraparound (code from PIDController)
        error = setpoint - angle
        if abs(error) > 180.0:
            if error > 0:
                error -= 360.0
            else:
                error += 360.0

        return abs(error) < self.kToleranceDegrees

    def reset_angle(self):
        self.navx.reset()

    def compute_error(self, setpoint, pid_input):
        """
        Compute the error between the setpoint and pid_input.
        :return: A value in degrees of error. (360 to -360)
        """
        error = pid_input - setpoint

        if abs(error) > 180.0:  # Used to find the closest path to the setpoint
            if error > 0:
                error -= 360.0
            else:
                error += 360.0

        return error

    def execute(self):
        super().execute()

        # rate will never be None when the component is not enabled
        if self.rate is not None:
            if self.is_aligned():
                self.stop()
            else:
                self.drivetrain.turn(self.rate, force=True)

    def stop(self):
        self.drivetrain.differential_drive(0)
