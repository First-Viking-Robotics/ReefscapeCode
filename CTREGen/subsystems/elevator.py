import math

import commands2
import rev
import wpilib
import wpilib.shuffleboard
import wpimath.controller
import wpimath.estimator
import wpimath.units
import wpimath.trajectory
import wpimath.system
import wpimath.system.plant
import constants


class Elevator(commands2.Subsystem):
    def __init__(self, constants=constants.Constants()):
        super().__init__()
        self.constants = constants
        self.profile = wpimath.trajectory.TrapezoidProfile(
            wpimath.trajectory.TrapezoidProfile.Constraints(
                self.constants.ElevatorMaxVelocity,
                self.constants.ElevatorMaxAccel,  # Max elevator speed and acceleration.
            )
        )

        self.PIDController = wpimath.controller.ProfiledPIDController(
            0.6, 0, 0, wpimath.trajectory.TrapezoidProfile.Constraints(
                self.constants.ElevatorMaxVelocity,
                self.constants.ElevatorMaxAccel
            )
        )

        self.motorFirst = rev.SparkMax(self.constants.ElevatorFirstMotorPort, rev.SparkMax.MotorType.kBrushless)
        self.motorSecond = rev.SparkMax(self.constants.ElevatorSecondMotorPort, rev.SparkMax.MotorType.kBrushless)
        self.motorFirst.setVoltage(12)
        self.motorSecond.setVoltage(12)

        # An encoder set up to measure flywheel velocity in radians per second.
        self.encoder = self.motorFirst.getAlternateEncoder()

        self.PIDController.reset(
            measuredPosition=self.encoder.getPosition(),
            measuredVelocity=self.encoder.getVelocity()
        )

        # Preset Goal
        self.goal = self.constants.ElevatorRest

        # Disable Motors at Start
        self.disable()
        self.encoder.setPosition(0)

        wpilib.SmartDashboard.putNumber("Elevator Goal", 0)

    def initMovement(self) -> None:

        # Reset our last reference to the current state.
        self.lastProfiledReference = wpimath.trajectory.TrapezoidProfile.State(
            self.encoder.getPosition(),
            wpimath.units.rotationsPerMinuteToRadiansPerSecond(self.encoder.getVelocity())
        )

    def goToL1(self):
        return commands2.cmd.runOnce(
            lambda: self._goToL1(), self
        )

    def goToL2(self):
        return commands2.cmd.runOnce(
            lambda: self._goToL2(), self
        )

    def goToL3(self):
        return commands2.cmd.runOnce(
            lambda: self._goToL3(), self
        )

    def goToL4(self):
        return commands2.cmd.runOnce(
            lambda: self._goToL4(), self
        )

    def goToRest(self):
        return commands2.cmd.runOnce(
            lambda: self._goToRest(), self
        )

    def _goToL1(self):
        self.goal = 1

    def _goToL2(self):
        self.goal = 2

    def _goToL3(self):
        self.goal = 3

    def _goToL4(self):
        self.goal = 4

    def _goToRest(self):
        self.goal = 0

    def disable(self):
        self.motorSecond.setVoltage(0)
        self.motorFirst.setVoltage(0)

    def periodic(self) -> None:
        # Sets the target position of our arm. This is similar to setting the setpoint of a
        # PID controller.

        percentageOutput = self.PIDController.calculate(
            self.encoder.getPosition(),
            wpilib.SmartDashboard.getNumber("Elevator Goal", 0)
        )

        wpilib.SmartDashboard.putNumber("Elevator Position", self.encoder.getPosition())
        wpilib.SmartDashboard.putNumber("Elevator PowerOutput", (percentageOutput + 0.1) * 0.75)

        # wpimath.filter.SlewRateLimiter(3)
        self.motorFirst.getClosedLoopController().setReference((percentageOutput + 0.1) * -0.75, rev.SparkMax.ControlType.kDutyCycle)
        self.motorSecond.getClosedLoopController().setReference((percentageOutput + 0.1) * -0.75, rev.SparkMax.ControlType.kDutyCycle)

    def atGoal(self):
        return (self.encoder.getPosition() == self.goal) and (self.encoder.getVelocity() == 0)
