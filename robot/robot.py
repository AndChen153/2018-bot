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
        # Drivetrain
        self.drivetrain_left_motor_master = ctre.WPI_TalonSRX(3)
        self.drivetrain_left_motor_slave = ctre.WPI_TalonSRX(4)
        self.drivetrain_right_motor_master = ctre.WPI_TalonSRX(5)
        self.drivetrain_right_motor_slave = ctre.WPI_TalonSRX(6)
        self.drivetrain_shifter_solenoid = wpilib.Solenoid(0)

        # Elevator
        self.elevator_motor = ctre.WPI_TalonSRX(8)
        self.elevator_solenoid = wpilib.DoubleSolenoid(1, 2)

        # Grabber
        self.grabber_left_motor = ctre.WPI_TalonSRX(2)
        self.grabber_right_motor = ctre.WPI_TalonSRX(1)

        # Controllers
        self.drive_controller = wpilib.XboxController(0)
        self.operator_controller = wpilib.XboxController(1)

        # Navx
        self.navx = navx.AHRS.create_spi()

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # Drive with controller
        self.drivetrain.differential_drive(
            self.drive_controller.getY(CONTROLLER_LEFT),
            self.drive_controller.getX(CONTROLLER_RIGHT))

        # Shifter - toggle into low gear when A button is pressed
        # for precise alignment. Otherwise stay high and zippy.
        if self.drive_controller.getAButtonReleased():
            self.drivetrain.shift_toggle()

        # Mirror elevator control & grabber control to both drive and
        # operator controllers to allow for driveteam flexibility.
        for controller in [self.drive_controller, self.operator_controller]:

            # Elevator position control
            if controller.getBumper(CONTROLLER_RIGHT):
                self.elevator.raise_to_switch()
            elif controller.getBumper(CONTROLLER_LEFT):
                self.elevator.lower_to_ground()

            # Elevator free control
            controller_pov = controller.getPOV(pov=0)
            if controller_pov == 0:
                self.elevator.raise_freely()
            elif controller_pov == 180:
                self.elevator.lower_freely()

            # Grabber
            if controller.getTriggerAxis(CONTROLLER_RIGHT):
                self.grabber.intake()
            elif controller.getTriggerAxis(CONTROLLER_LEFT):
                self.grabber.deposit()


if __name__ == '__main__':
    wpilib.run(SpartaBot)
