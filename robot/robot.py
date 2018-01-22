import magicbot
import wpilib
import ctre

from components import drivetrain, elevator, grabber


class SpartaBot(magicbot.MagicRobot):

    drivetrain = drivetrain.Drivetrain
    elevator = elevator.Elevator
    grabber = grabber.Grabber

    def createObjects(self):
        '''Create motors and stuff here'''

        # Drivetrain
        self.drivetrain_left_motor_master = ctre.CANTalon(5)
        self.drivetrain_left_motor_slave = ctre.CANTalon(10)
        self.drivetrain_right_motor_master = ctre.CANTalon(15)
        self.drivetrain_right_motor_slave = ctre.CANTalon(20)
        self.drivetrain_shifter_solenoid = wpilib.Solenoid(1)

        # Elevator
        self.elevator_motor = ctre.CANTalon(25)
        self.elevator_solenoid = wpilib.DoubleSolenoid(2)

        # Grabber
        self.grabber_left_motor = ctre.CANTalon(1)
        self.grabber_right_motor = ctre.CANTalon(2)

        # Controller
        self.drive_controller = wpilib.XboxController(0)

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        # Drive with controller
        self.drivetrain.differential_drive(
            self.drive_controller.getY(wpilib.XboxController.Hand.kLeft),
            self.drive_controller.getX(wpilib.XboxController.Hand.kRight))

        # Shifter - switch into low gear when A button is held
        # for precise targeting. Otherwise stay high and zippy.
        if self.drive_controller.getAButton():
            self.drivetrain.shift_low_gear()
        else:
            self.drivetrain.shift_high_gear()

        # Elevator position control
        if self.drive_controller.getRightBumper():
            self.elevator.raise_to_switch()
        elif self.drive_controller.getLeftBumper():
            self.elevator.lower_to_ground()

        # Elevator free control
        controller_pov = self.drive_controller.getPOV()
        if controller_pov == 90:
            self.elevator.raise_freely()
        elif controller_pov == 180:
            self.elevator.lower_freely()

        # Grabber
        if self.drive_controller.getRightTrigger():
            self.grabber.intake()
        elif self.drive_controller.getLeftTrigger():
            self.grabber.deposit()


if __name__ == '__main__':
    wpilib.run(SpartaBot)
