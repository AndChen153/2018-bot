import magicbot
import wpilib
import ctre
from robotpy_ext.common_drivers import navx
from components import drivetrain, elevator, grabber, field
from common import xbox_updater, util
from controllers import position_controller, angle_controller, \
    trajectory_controller, grabber_orienter_controller

CONTROLLER_LEFT = wpilib.XboxController.Hand.kLeft
CONTROLLER_RIGHT = wpilib.XboxController.Hand.kRight


class SpartaBot(magicbot.MagicRobot):

    drivetrain = drivetrain.Drivetrain
    elevator = elevator.Elevator
    grabber = grabber.Grabber
    field = field.Field

    position_controller = position_controller.PositionController
    angle_controller = angle_controller.AngleController
    trajectory_controller = trajectory_controller.TrajectoryController
    grabber_orienter_controller = grabber_orienter_controller. \
        GrabberOrienterController

    def createObjects(self):
        # Drivetrain
        self.drivetrain_left_motor_master = ctre.WPI_TalonSRX(4)
        self.drivetrain_left_motor_slave = ctre.WPI_TalonSRX(3)
        self.drivetrain_right_motor_master = ctre.WPI_TalonSRX(5)
        self.drivetrain_right_motor_slave = ctre.WPI_TalonSRX(6)
        self.drivetrain_shifter_solenoid = wpilib.Solenoid(0)

        # Elevator
        self.elevator_motor = ctre.WPI_TalonSRX(8)
        self.elevator_solenoid = wpilib.DoubleSolenoid(1, 2)
        self.elevator_reverse_limit = wpilib.DigitalInput(0)

        # Grabber
        self.grabber_left_motor = ctre.WPI_TalonSRX(2)
        self.grabber_right_motor = ctre.WPI_TalonSRX(1)

        # Controllers
        self.drive_controller = wpilib.XboxController(0)
        self.operator_controller = wpilib.XboxController(1)

        # Initialize dashboard
        xbox_updater.push(self.drive_controller, 'driver')
        xbox_updater.push(self.operator_controller, 'operator')

        # Navx
        self.navx = navx.AHRS.create_spi()

    def teleopInit(self):
        self.drivetrain.reset_angle_correction()
        self.position_controller.reset_position_and_heading()

    def teleopPeriodic(self):
        # Drive with controller
        angle = self.drive_controller.getX(CONTROLLER_RIGHT)
        if not self.drive_controller.getStickButtonPressed(CONTROLLER_RIGHT):
            # Unless the right stick is pressed down, scale down turning inputs
            # to keep the robot from flying around the field / overshooting.
            angle = util.scale(angle, -1, 1, -0.75, 0.75)
        self.drivetrain.differential_drive(
            self.drive_controller.getY(CONTROLLER_LEFT), angle)

        # Shifter - toggle into low gear when bumper pressed
        # for precise alignment. Otherwise stay high and zippy.
        if self.drive_controller.getBumperReleased(CONTROLLER_LEFT):
            self.drivetrain.shift_toggle()

        # Mirror elevator control & grabber control to both drive and
        # operator controllers to allow for driveteam flexibility.
        for controller in [self.drive_controller, self.operator_controller]:

            # Elevator position control
            if controller.getYButton():
                self.elevator.raise_to_switch()
            elif controller.getAButton():
                self.elevator.lower_to_ground()

            # Elevator free control
            controller_pov = controller.getPOV(pov=0)
            if controller_pov == 0:
                self.elevator.raise_freely()
            elif controller_pov == 180:
                self.elevator.lower_freely()

            # Elevator incremental control
            if controller_pov == 90:
                self.elevator.move_incremental(1)
            elif controller_pov == 270:
                self.elevator.move_incremental(-1)

            # Grabber - right trigger for flippy & full intake,
            # left trigger to deposit cube.
            if controller.getTriggerAxis(CONTROLLER_RIGHT):
                self.grabber.intake()
            elif controller.getTriggerAxis(CONTROLLER_LEFT):
                self.grabber_orienter_controller.orient(
                    grabber_orienter_controller.GrabberOrienterSide.FLIPPY)
            elif controller.getBumper(CONTROLLER_RIGHT):
                self.grabber.deposit()

            # Position testing
            # if controller.getBButton():
            #     self.position_controller.move_to(36)

        # Pass inputs to dashboard
        xbox_updater.push(self.drive_controller, 'driver')
        xbox_updater.push(self.operator_controller, 'operator')

    def disabledPeriodic(self):
        # Pass inputs to dashboard
        xbox_updater.push(self.drive_controller, 'driver')
        xbox_updater.push(self.operator_controller, 'operator')


if __name__ == '__main__':
    wpilib.run(SpartaBot)
