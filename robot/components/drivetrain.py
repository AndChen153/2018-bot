import math
from collections import namedtuple
from ctre import WPI_TalonSRX
from wpilib import RobotDrive, Solenoid
from magicbot import tunable
from robotpy_ext.common_drivers import navx

from constants import TALON_TIMEOUT
from common import util

DifferentialDriveConfig = namedtuple('DifferentialDriveConfig',
                                     ['y', 'rotation', 'squared'])

HIGH_GEAR = False
LOW_GEAR = True

UNITS_PER_REV = 4096
DISTANCE_PER_REV = (2 * math.pi * 3) / (3 / 1) / (54 / 30)


class Drivetrain:

    navx = navx.AHRS

    left_motor_master = WPI_TalonSRX
    left_motor_slave = WPI_TalonSRX
    right_motor_master = WPI_TalonSRX
    right_motor_slave = WPI_TalonSRX

    shifter_solenoid = Solenoid

    angle_correction_factor = tunable(0.05)
    angle_correction_max = tunable(0.1)

    def setup(self):
        self.pending_differential_drive = None
        self.force_differential_drive = False
        self.pending_gear = HIGH_GEAR
        self.pending_position = None
        self.pending_reset = False

        # Set encoders
        self.left_motor_master.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.right_motor_master.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.left_motor_master.setSensorPhase(True)

        # Set slave motors
        self.left_motor_slave.set(WPI_TalonSRX.ControlMode.Follower,
                                  self.left_motor_master.getDeviceID())
        self.right_motor_slave.set(WPI_TalonSRX.ControlMode.Follower,
                                   self.right_motor_master.getDeviceID())

        # Set up drive control
        self.robot_drive = RobotDrive(self.left_motor_master,
                                      self.right_motor_master)

    def reset_position(self):
        self.pending_reset = True

    def get_position(self):
        '''
        Returns averaged quadrature position in inches.
        '''
        left_position = -self.left_motor_master.getQuadraturePosition()
        right_position = self.right_motor_master.getQuadraturePosition()
        print('drivetrain pos', 'left', left_position, 'right', right_position)
        return (((left_position + right_position) / 2) *
                (1 / UNITS_PER_REV) * DISTANCE_PER_REV)

    def differential_drive(self, y, rotation=0, squared=True, force=False):
        if not self.force_differential_drive:
            self.pending_differential_drive = DifferentialDriveConfig(
                y=y, rotation=rotation, squared=squared)
            self.force_differential_drive = force

    def turn(self, rotation=0, force=False):
        self.differential_drive(0, rotation, squared=False, force=force)

    def reset_angle_correction(self):
        self.navx.reset()

    def angle_corrected_differential_drive(self, y, rotation=0):
        '''
        Heading must be reset first. (drivetrain.reset_angle_correction())
        '''
        heading = self.navx.getYaw()
        correction = util.abs_clamp(-self.angle_correction_factor * heading,
                                    0, self.angle_correction_max)
        self.differential_drive(y, rotation + correction)

    def shift_low_gear(self):
        self.pending_gear = LOW_GEAR

    def shift_high_gear(self):
        self.pending_gear = HIGH_GEAR

    def shift_toggle(self):
        if self.pending_gear == HIGH_GEAR:
            self.pending_gear = LOW_GEAR
        else:
            self.pending_gear = HIGH_GEAR

    def execute(self):
        # Reset position
        if self.pending_reset:
            self.left_motor_master.setQuadraturePosition(0, TALON_TIMEOUT)
            self.right_motor_master.setQuadraturePosition(0, TALON_TIMEOUT)
            self.pending_reset = False

        # Drive
        if self.pending_differential_drive:
            self.robot_drive.arcadeDrive(
                self.pending_differential_drive.y,
                self.pending_differential_drive.rotation,
                squaredInputs=self.pending_differential_drive.squared)
            self.pending_differential_drive = None
            self.force_differential_drive = False

        # Shifter
        self.shifter_solenoid.set(self.pending_gear)

    def on_disabled(self):
        self.robot_drive.drive(0, 0)
