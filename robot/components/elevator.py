from magicbot import tunable
from ctre import CANTalon
from enum import IntEnum
from wpilib import DoubleSolenoid


class ElevatorState(IntEnum):
    RETRACTED = 0
    DEPLOYED = 1


class ElevatorPosition(IntEnum):
    GROUND = 0
    SWITCH = 10


class Elevator:

    motor = CANTalon
    solenoid = DoubleSolenoid

    speed = tunable(1)

    def setup(self):
        self.pending_state = None
        self.pending_position = None
        self.pending_drive = None
        # self.motor.enableLimitSwitch(True, True)  # Top/bottom

    def deploy(self):
        self.pending_state = ElevatorState.DEPLOYED

    def retract(self):
        self.pending_state = ElevatorState.RETRACTED

    def raise_to_switch(self):
        self.pending_position = ElevatorPosition.SWITCH

    def lower_to_ground(self, position):
        self.pending_position = ElevatorPosition.GROUND

    def raise_freely(self):
        self.pending_drive = self.speed

    def lower_freely(self):
        self.pending_drive = -self.speed

    def execute(self):
        # Elevator motor
        if self.pending_drive:
            self.motor.changeControlMode(CANTalon.ControlMode.PercentVbus)
            self.motor.set(self.pending_drive)
            self.pending_drive = None
        elif self.pending_position:
            self.motor.changeControlMode(CANTalon.ControlMode.Position)
            self.motor.set(self.pending_position)
            self.pending_position = None

        # Elevator deployment/retraction
        if self.pending_state:
            if self.pending_state == ElevatorState.RETRACTED:
                self.solenoid.set(DoubleSolenoid.Value.kForward)
            elif self.pending_state == ElevatorState.DEPLOYED:
                self.solenoid.set(DoubleSolenoid.kReverse)
            self.pending_state = None
