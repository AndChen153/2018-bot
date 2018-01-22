from collections import namedtuple
from ctre import CANTalon
from wpilib import RobotDrive, Solenoid

DifferentialDriveConfig = namedtuple('DifferentialDriveConfig',
                                     ['y', 'rotation'])

HIGH_GEAR = True
LOW_GEAR = False


class Drivetrain:

    left_motor_master = CANTalon
    left_motor_slave = CANTalon
    right_motor_master = CANTalon
    right_motor_slave = CANTalon

    shifter_solenoid = Solenoid

    def setup(self):
        self.pending_differential_drive = None
        self.pending_gear = HIGH_GEAR

        # Set slave motors
        self.left_motor_slave.changeControlMode(CANTalon.ControlMode.Follower)
        self.right_motor_slave.changeControlMode(CANTalon.ControlMode.Follower)
        self.left_motor_slave.set(self.left_motor_master.getDeviceID())
        self.right_motor_slave.set(self.right_motor_master.getDeviceID())

        # Set up drive control
        self.robot_drive = RobotDrive(self.left_motor_master,
                                      self.right_motor_master)

    def differential_drive(self, y, rotation):
        self.pending_differential_drive = DifferentialDriveConfig(
            y=y, rotation=rotation)

    def shift_low_gear(self):
        self.pending_gear = LOW_GEAR

    def shift_high_gear(self):
        self.pending_gear = HIGH_GEAR

    def execute(self):
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
