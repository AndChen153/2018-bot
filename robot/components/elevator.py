import math
from magicbot import tunable
from ctre import WPI_TalonSRX
from enum import IntEnum
from wpilib import DoubleSolenoid

from constants import TALON_TIMEOUT

# 12T #35 sprocket diameter: https://www.vexrobotics.com/35-sprockets.html
UNITS_PER_REV = 4096
DISTANCE_PER_REV = math.pi * 1.786  # pi * sprocket diameter


class ElevatorState(IntEnum):
    RETRACTED = 0
    DEPLOYED = 1


class ElevatorPosition(IntEnum):
    GROUND = 0
    SWITCH = 36 / DISTANCE_PER_REV * UNITS_PER_REV  # 36 inches


class Elevator:

    motor = WPI_TalonSRX
    solenoid = DoubleSolenoid

    kFreeSpeed = tunable(0.6)
    kZeroingSpeed = tunable(0.2)
    kP = tunable(0.0)
    kI = tunable(0.0)
    kD = tunable(0.0)
    kF = tunable(0.0)

    def setup(self):
        self.pending_state = None
        self.pending_position = None
        self.pending_drive = None

        self.has_zeroed = False

        self.motor.setInverted(True)
        self.motor.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.motor.selectProfileSlot(0, 0)
        self.motor.setSensorPhase(True)
        self.motor.configReverseLimitSwitchSource(0, True, 0)
        self.motor.config_kP(0, self.kP, 0)
        self.motor.config_kI(0, self.kI, 0)
        self.motor.config_kD(0, self.kD, 0)
        self.motor.config_kF(0, self.kF, 0)

    def is_encoder_connected(self):
        return self.motor.getPulseWidthRiseToRiseUs() != 0

    def deploy(self):
        self.pending_state = ElevatorState.DEPLOYED

    def retract(self):
        self.pending_state = ElevatorState.RETRACTED

    def raise_to_switch(self):
        self.pending_position = ElevatorPosition.SWITCH

    def lower_to_ground(self):
        self.pending_position = ElevatorPosition.GROUND

    def raise_freely(self):
        self.pending_drive = self.kFreeSpeed

    def lower_freely(self):
        self.pending_drive = -self.kFreeSpeed

    def execute(self):
        # Elevator motor
        if self.pending_drive:
            self.motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                           self.pending_drive)
            self.pending_drive = None
            self.pending_position = None  # Clear old pending position

        elif self.pending_position and self.is_encoder_connected():
            # Note: we don't clear the pending position so that we keep
            # on driving to the position in subsequent execute() cycles.
            if not self.has_zeroed and \
                    self.pending_position == ElevatorPosition.GROUND:
                # Drive downwards until we zero it... and cross our fingers...
                self.motor.set(WPI_TalonSRX.ControlMode.PercentOutput,
                               -self.kZeroingSpeed)
            else:
                self.motor.set(WPI_TalonSRX.ControlMode.Position,
                               self.pending_position)

        else:
            if self.is_encoder_connected():
                # If no command, hold position in place (basically, a more
                # "aggressive" brake mode to prevent any slippage).
                self.pending_position = self.motor.getQuadraturePosition()
            else:
                self.motor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0)

        # Elevator deployment/retraction
        if self.pending_state:
            if self.pending_state == ElevatorState.RETRACTED:
                self.solenoid.set(DoubleSolenoid.Value.kForward)
            elif self.pending_state == ElevatorState.DEPLOYED:
                self.solenoid.set(DoubleSolenoid.kReverse)
            self.pending_state = None

        # Zero the encoder when hits bottom of elevator
        if self.motor.isRevLimitSwitchClosed():
            self.motor.setQuadraturePosition(0, TALON_TIMEOUT)
            self.has_zeroed = True
