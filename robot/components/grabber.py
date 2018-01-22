from magicbot import tunable
from ctre import CANTalon
from enum import IntEnum


class GrabberState(IntEnum):
    DISABLED = 0
    INTAKING = 1
    DEPOSITING = 2


class Grabber:

    intake_speed = tunable(1)
    deposit_speed = tunable(1)

    left_motor = CANTalon
    right_motor = CANTalon

    def setup(self):
        self.pending_state = GrabberState.DISABLED
        self.right_motor.setInverted(True)
        self.right_motor.changeControlMode(CANTalon.ControlMode.Follower)
        self.right_motor.set(self.left_motor.getDeviceID())

    def intake(self):
        self.pending_state = GrabberState.INTAKING

    def deposit(self):
        self.pending_state = GrabberState.DEPOSITING

    def execute(self):
        if self.pending_state == GrabberState.DISABLED:
            self.left_motor.set(0)
        elif self.pending_state == GrabberState.INTAKING:
            self.left_motor.set(self.intake_speed)
        elif self.pending_state == GrabberState.DEPOSITING:
            self.left_motor.set(self.deposit_speed)

        self.pending_state = GrabberState.DISABLED

    def on_disabled(self):
        self.left_motor.set(0)
