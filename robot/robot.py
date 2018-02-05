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
        self.drivetrain.reset_angle_correction()

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
            # if controller.getBumper(CONTROLLER_RIGHT):
            #     self.elevator.raise_to_switch()
            # elif controller.getBumper(CONTROLLER_LEFT):
            #     self.elevator.lower_to_ground()

            # Elevator free control
            controller_pov = controller.getPOV(pov=0)
            if controller_pov == 0:
                self.elevator.raise_freely()
            elif controller_pov == 180:
                self.elevator.lower_freely()

            # Grabber - right trigger for flippy & full intake,
            # left trigger to deposit cube.
            right_trigger = controller.getTriggerAxis(CONTROLLER_RIGHT)
            if right_trigger > 0.75:
                self.grabber.intake()
            elif right_trigger > 0:
                self.grabber_orienter_controller.orient(
                    grabber_orienter_controller.GrabberOrienterSide.FLIPPY)
            elif controller.getTriggerAxis(CONTROLLER_LEFT):
                self.grabber.deposit()

        # Pass inputs to dashboard
        xbox_updater.push(self.drive_controller, 'driver')
        xbox_updater.push(self.operator_controller, 'operator')


if __name__ == '__main__':
    wpilib.run(SpartaBot)
