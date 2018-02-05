from magicbot import tunable
from ctre import WPI_TalonSRX
from enum import IntEnum


class GrabberState(IntEnum):
    DISABLED = 0
    INTAKING = 1
    DEPOSITING = 2


class Grabber:

    intake_speed = tunable(0.8)
    deposit_speed = tunable(0.5)
    current_limit = tunable(0.0)

    left_motor = WPI_TalonSRX
    right_motor = WPI_TalonSRX

    def setup(self):
        self.pending_state = GrabberState.DISABLED
        self.pending_independent_control = None

        self.right_motor.setInverted(True)

    def intake(self):
        self.pending_state = GrabberState.INTAKING

    def deposit(self):
        self.pending_state = GrabberState.DEPOSITING

    def control_independently(self, left, right):
        self.pending_independent_control = [left, right]

    def execute(self):
        # Independent motor control - bypass all the normal controls
        if self.pending_independent_control:
            self.left_motor.set(self.pending_independent_control[0])
            self.right_motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                                 self.pending_independent_control[1])
            self.pending_independent_control = None
            return

        self.right_motor.set(WPI_TalonSRX.ControlMode.Follower,
                             self.left_motor.getDeviceID())

        if self.pending_state == GrabberState.DISABLED:
            self.left_motor.set(0)
        elif self.pending_state == GrabberState.INTAKING:
            self.left_motor.set(self.intake_speed)
        elif self.pending_state == GrabberState.DEPOSITING:
            self.left_motor.set(-self.deposit_speed)

        self.pending_state = GrabberState.DISABLED

    def on_disabled(self):
        self.left_motor.set(0)
