import math
from collections import namedtuple
from ctre import WPI_TalonSRX
from wpilib import RobotDrive, Solenoid

DifferentialDriveConfig = namedtuple('DifferentialDriveConfig',
                                     ['y', 'rotation'])

HIGH_GEAR = True
LOW_GEAR = False

UNITS_PER_REV = 4096
DISTANCE_PER_REV = 2 * math.pi * 6


class Drivetrain:

    left_motor_master = WPI_TalonSRX
    left_motor_slave = WPI_TalonSRX
    right_motor_master = WPI_TalonSRX
    right_motor_slave = WPI_TalonSRX

    shifter_solenoid = Solenoid

    def setup(self):
        self.pending_differential_drive = None
        self.pending_gear = HIGH_GEAR
        self.pending_position = None
        self.pending_reset = False

        # Set encoders
        self.left_motor_master.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.right_motor_master.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)

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
        left_position = self.left_motor_master.getQuadraturePosition()
        right_position = self.right_motor_master.getQuadraturePosition()
        return (((left_position + right_position) / 2) *
                (1 / UNITS_PER_REV) * DISTANCE_PER_REV)

    def differential_drive(self, y, rotation=0):
        self.pending_differential_drive = DifferentialDriveConfig(
            y=y, rotation=rotation)

    def shift_low_gear(self):
        self.pending_gear = LOW_GEAR

    def shift_high_gear(self):
        self.pending_gear = HIGH_GEAR

    def execute(self):
        # Reset position
        if self.pending_reset:
            self.left_motor_master.setQuadraturePosition(0, 0)
            self.right_motor_master.setQuadraturePosition(0, 0)
            self.pending_reset = False

        # Drive
        if self.pending_differential_drive:
            self.robot_drive.arcadeDrive(
                self.pending_differential_drive.y,
                self.pending_differential_drive.rotation)
            self.pending_differential_drive = None

        # Shifter
        self.shifter_solenoid.set(self.pending_gear)

    def on_disabled(self):
        self.robot_drive.drive(0, 0)
