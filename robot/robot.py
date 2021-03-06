import magicbot
import wpilib
import ctre
import hal
from robotpy_ext.common_drivers import navx
from components import bot, drivetrain, elevator, grabber, field, \
    targeting, ramp, macros
from common import rumbler
from controllers import position_controller, angle_controller, \
    trajectory_controller, grabber_orienter_controller, \
    grabber_auto_controller, cube_hunter_controller, \
    hold_position_controller, macro_controller, path_controller, \
    path_recorder_controller

CONTROLLER_LEFT = wpilib.XboxController.Hand.kLeft
CONTROLLER_RIGHT = wpilib.XboxController.Hand.kRight

USE_PATH_RECORDER = False


class SpartaBot(magicbot.MagicRobot):

    # Macro controller - load before all other components to ensure
    # time to inject / poll before execute() method called.
    # Components are loaded alphabetically, so...
    aaa_macro_controller = macro_controller.MacroController

    macros = macros.Macros
    bot = bot.Bot
    drivetrain = drivetrain.Drivetrain
    elevator = elevator.Elevator
    grabber = grabber.Grabber
    ramp = ramp.Ramp
    targeting = targeting.Targeting

    position_controller = position_controller.PositionController
    angle_controller = angle_controller.AngleController
    trajectory_controller = trajectory_controller.TrajectoryController
    grabber_orienter_controller = grabber_orienter_controller. \
        GrabberOrienterController
    grabber_auto_controller = grabber_auto_controller. \
        GrabberAutoController
    cube_hunter_controller = cube_hunter_controller.CubeHunterController
    hold_position_controller = hold_position_controller.HoldPositionController
    path_controller = path_controller.PathController
    path_recorder_controller = path_recorder_controller.PathRecorderController

    field = field.Field

    def createObjects(self):
        # TEMP - PRACTICE BOT ONLY
        # Launch cameraserver
        # wpilib.CameraServer.launch()

        # Practice bot
        # On practice bot, DIO is shorted
        # self.is_practice_bot = wpilib.DigitalInput(30)
        if hal.HALIsSimulation():
            self.is_practice_bot = wpilib.DigitalInput(20)
        else:
            self.is_practice_bot = wpilib.DigitalInput(30)

        # Drivetrain
        self.drivetrain_left_motor_master = ctre.WPI_TalonSRX(4)
        self.drivetrain_left_motor_slave = ctre.WPI_TalonSRX(7)  # was 3
        self.drivetrain_right_motor_master = ctre.WPI_TalonSRX(5)
        self.drivetrain_right_motor_slave = ctre.WPI_TalonSRX(6)
        self.drivetrain_shifter_solenoid = wpilib.Solenoid(0)

        # Elevator
        self.elevator_motor = ctre.WPI_TalonSRX(8)
        self.elevator_solenoid = wpilib.DoubleSolenoid(1, 2)
        self.elevator_reverse_limit = wpilib.DigitalInput(0)

        # Grabber
        self.grabber_left_motor = ctre.WPI_TalonSRX(1)
        self.grabber_right_motor = ctre.WPI_TalonSRX(2)

        # Ramp
        self.ramp_solenoid = wpilib.DoubleSolenoid(3, 4)
        self.ramp_motor = ctre.WPI_TalonSRX(3)  # was 7
        self.hold_start_time = None

        # Controllers
        self.drive_controller = wpilib.XboxController(0)
        self.operator_controller = wpilib.XboxController(1)

        # Compressor
        self.compressor = wpilib.Compressor()

        # LEDs
        self.led_driver = wpilib.Spark(0)

        # Navx
        self.navx = navx.AHRS.create_spi()

        # Macros / path recorder
        self._is_recording_macro = False
        self._is_playing_macro = False
        self._is_recording_path = False

        # Flippy toggle
        self._is_flippy = False

    def teleopInit(self):
        self.elevator.release_lock()
        self.drivetrain.reset_angle_correction()
        self.position_controller.reset_position_and_heading()
        self.trajectory_controller.reset()
        self.grabber_auto_controller.enable()
        self.drivetrain.shift_high_gear()

    def teleopPeriodic(self):
        # Drive with controller
        angle = self.drive_controller.getX(CONTROLLER_RIGHT)
        self.drivetrain.angle_corrected_differential_drive(
            self.drive_controller.getY(CONTROLLER_LEFT), angle)

        # Shifter - toggle on bumper press
        if self.drive_controller.getBumperReleased(CONTROLLER_LEFT):
            self.drivetrain.shift_toggle()

        for controller in [self.drive_controller]:

            # Elevator position control
            if controller.getYButton():
                self.elevator.raise_to_switch()
            elif controller.getAButton():
                self.elevator.lower_to_ground()
            elif controller.getBButton():
                self.elevator.raise_to_carry()

            if controller.getBumperReleased(CONTROLLER_RIGHT):
                self.elevator.toggle_carry_ground()

            # Elevator free control
            controller_pov = controller.getPOV(pov=0)
            if controller_pov == 0:
                self.elevator.raise_freely()
            elif controller_pov == 180:
                self.elevator.lower_freely()

            # Grabber - right trigger for full intake, x button for flippy,
            # left trigger to deposit cube.
            if controller.getTriggerAxis(CONTROLLER_RIGHT):
                self.grabber.intake()
            elif controller.getTriggerAxis(CONTROLLER_LEFT):
                self.grabber.deposit()
            elif controller.getXButtonReleased():
                self._is_flippy = not self._is_flippy

            if self._is_flippy and self.elevator.is_at_ground():
                self.grabber_orienter_controller.orient(
                    grabber_orienter_controller.GrabberOrienterSide.FLIPPY)

            # Ramp deployment
            if controller.getStartButton() and controller.getBackButton() and \
                    not self.ramp.is_released():
                rumbler.rumble(controller, 1)
                if not self.hold_start_time:
                    self.hold_start_time = wpilib.Timer.getFPGATimestamp()
                elif wpilib.Timer.getFPGATimestamp() - self.hold_start_time \
                        > ramp.SAFETY_RELEASE_WAIT:
                    self.drivetrain.shift_low_gear()
                    self.ramp.release()
                    rumbler.rumble(controller, 0)
                    self.hold_start_time = None

            # Ramp raising / lowering
            if not controller.getBackButton() and controller.getStartButton():
                self.ramp.raise_ramp()
            if controller.getBackButton() and not controller.getStartButton():
                self.ramp.lower_ramp()

            # Ramp hold position
            if self.ramp.is_released() and not \
                    (self.drive_controller.getStickButtonPressed(
                        CONTROLLER_RIGHT) or
                        self.drive_controller.getStickButtonPressed(
                            CONTROLLER_LEFT)):
                self.hold_position_controller.engage()

            # Path recorder
            if USE_PATH_RECORDER:
                if self.operator_controller.getAButtonReleased():
                    self._is_recording_path = not self._is_recording_path
                if self._is_recording_path:
                    self.path_recorder_controller.record()

    def disabledPeriodic(self):
        # Lock ramps
        self.ramp.lock()


if __name__ == '__main__':
    wpilib.run(SpartaBot)
