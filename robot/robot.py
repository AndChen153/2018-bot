import magicbot
import wpilib
import ctre
from robotpy_ext.common_drivers import navx

from components import drivetrain, elevator, grabber, field
from controllers import position_controller, angle_controller

CONTROLLER_LEFT = wpilib.XboxController.Hand.kLeft
CONTROLLER_RIGHT = wpilib.XboxController.Hand.kRight


class SpartaBot(magicbot.MagicRobot):

    drivetrain = drivetrain.Drivetrain
    elevator = elevator.Elevator
    grabber = grabber.Grabber
    field = field.Field

    position_controller = position_controller.PositionController
    angle_controller = angle_controller.AngleController

    def createObjects(self):
        '''Create motors and stuff here'''

        # Drivetrain
        self.drivetrain_left_motor_master = ctre.WPI_TalonSRX(5)
        self.drivetrain_left_motor_slave = ctre.WPI_TalonSRX(10)
        self.drivetrain_right_motor_master = ctre.WPI_TalonSRX(15)
        self.drivetrain_right_motor_slave = ctre.WPI_TalonSRX(20)
        self.drivetrain_shifter_solenoid = wpilib.Solenoid(1)

        # Elevator
        self.elevator_motor = ctre.WPI_TalonSRX(25)
        self.elevator_solenoid = wpilib.DoubleSolenoid(2, 3)

        # Grabber
        self.grabber_left_motor = ctre.WPI_TalonSRX(1)
        self.grabber_right_motor = ctre.WPI_TalonSRX(2)

        # Controller
        self.drive_controller = wpilib.XboxController(0)

        # Navx
        self.navx = navx.AHRS.create_spi()

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # Drive with controller
        self.drivetrain.differential_drive(
            self.drive_controller.getY(CONTROLLER_LEFT),
            self.drive_controller.getX(CONTROLLER_RIGHT))

        # Shifter - switch into low gear when A button is held
        # for precise targeting. Otherwise stay high and zippy.
        if self.drive_controller.getAButton():
            self.drivetrain.shift_low_gear()
        else:
            self.drivetrain.shift_high_gear()

        # Elevator position control
        if self.drive_controller.getBumper(CONTROLLER_RIGHT):
            self.elevator.raise_to_switch()
        elif self.drive_controller.getBumper(CONTROLLER_LEFT):
            self.elevator.lower_to_ground()

        # Elevator free control
        controller_pov = self.drive_controller.getPOV(pov=0)
        if controller_pov == 90:
            self.elevator.raise_freely()
        elif controller_pov == 180:
            self.elevator.lower_freely()

        # Grabber
        if self.drive_controller.getTriggerAxis(CONTROLLER_RIGHT):
            self.grabber.intake()
        elif self.drive_controller.getTriggerAxis(CONTROLLER_LEFT):
            self.grabber.deposit()


if __name__ == '__main__':
    wpilib.run(SpartaBot)
