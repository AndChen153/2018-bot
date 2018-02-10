from magicbot import tunable
from ctre import WPI_TalonSRX
from wpilib import DoubleSolenoid
from enum import IntEnum


# Time in seconds to wait after buttons are held before deploying the ramps.
SAFETY_RELEASE_WAIT = 2


class RampState(IntEnum):
    LOCKED = 0
    RELEASED = 1


class WinchState(IntEnum):
    DISABLED = 0
    RAISING = 1


class Ramp:

    solenoid = DoubleSolenoid
    motor = WPI_TalonSRX

    speed = tunable(1.0)

    def setup(self):
        self.state = RampState.LOCKED
        self.pending_winch = WinchState.DISABLED
        self.motor.setInverted(True)

    def release(self):
        self.state = RampState.RELEASED

    def is_released(self):
        return self.state == RampState.RELEASED

    def lock(self):
        self.state = RampState.LOCKED

    def raise_ramp(self):
        self.pending_winch = WinchState.RAISING

    def execute(self):
        # Winch
        if self.pending_winch == WinchState.RAISING:
            self.motor.set(self.speed)
            self.pending_winch = WinchState.DISABLED
        else:
            self.motor.set(0)

        # Solenoid lock
        if self.state == RampState.LOCKED:
            self.solenoid.set(DoubleSolenoid.Value.kForward)
        elif self.state == RampState.RELEASED:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)

    def on_disabled(self):
        self.motor.set(0)
