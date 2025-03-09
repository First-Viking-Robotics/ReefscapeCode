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


class Elevator(commands2.Subsystem):
    def __init__(self, enabled):
        super().__init__()
        
        self.PIDController = wpimath.controller.ProfiledPIDController(
            0.6, 0, 0, wpimath.trajectory.TrapezoidProfile.Constraints(
                0.7,
                0.7
            )
        )

        self.enabled = enabled

        self.motorFirst = rev.SparkMax(26, rev.SparkMax.MotorType.kBrushless)
        self.motorSecond = rev.SparkMax(27, rev.SparkMax.MotorType.kBrushless)
        self.motorFirst.setVoltage(12)
        self.motorSecond.setVoltage(12)

        # An encoder set up to measure flywheel velocity in radians per second.
        self.encoder = self.motorFirst.getAlternateEncoder()

        self.PIDController.reset(
            measuredPosition=self.encoder.getPosition(),
            measuredVelocity=self.encoder.getVelocity()
        )

        # Preset Goal
        self.goal = 0

        # Disable Motors at Start
        self.disable()
        self.encoder.setPosition(0)

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

    def periodic(self):
        return commands2.cmd.run(
            lambda: self._periodic(), self
            ).ignoringDisable(True)

    def _goToL1(self):
        self.goal = 0.01

    def _goToL2(self):
        self.goal = 0.3

    def _goToL3(self):
        self.goal = 1.7

    def _goToL4(self):
        self.goal = 4

    def _goToRest(self):
        self.goal = 0

    def disable(self):
        self.motorFirst.getClosedLoopController().setReference(0, rev.SparkMax.ControlType.kDutyCycle)
        self.motorSecond.getClosedLoopController().setReference(0, rev.SparkMax.ControlType.kDutyCycle)

    def _periodic(self) -> None:
        # Sets the target position of our arm. This is similar to setting the setpoint of a
        # PID controller.

        percentageOutput = self.PIDController.calculate(
            self.encoder.getPosition(),
            self.goal
        )

        wpilib.SmartDashboard.putNumber("Elevator Position", self.encoder.getPosition())

        # wpilib.SmartDashboard().putValue("Elevator Value", self.encoder.getPosition())

        # wpimath.filter.SlewRateLimiter(3)
        if self.enabled:
            self.motorFirst.getClosedLoopController().setReference((percentageOutput + 0.29) * -0.75, rev.SparkMax.ControlType.kDutyCycle)
            self.motorSecond.getClosedLoopController().setReference((percentageOutput + 0.29) * -0.75, rev.SparkMax.ControlType.kDutyCycle)
            wpilib.SmartDashboard.putNumber("Elevator Power", (percentageOutput + 0.29) * -0.75)

    def atGoal(self):
        return (self.encoder.getPosition() == self.goal) and (self.encoder.getVelocity() == 0)
