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
        return ElevatorToL1(self)

    def goToL2(self):
        return ElevatorToL2(self)

    def goToL3(self):
        return ElevatorToL3(self)

    def goToL4(self):
        return ElevatorToL4(self)

    def goToRest(self):
        return ElevatorToRest(self)

    def _goToL1(self):
        self.goal = self.constants.ElevatorL1

    def _goToL2(self):
        self.goal = self.constants.ElevatorL2

    def _goToL3(self):
        self.goal = self.constants.ElevatorL3

    def _goToL4(self):
        self.goal = self.constants.ElevatorL4

    def _goToRest(self):
        self.goal = self.constants.ElevatorRest

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
        wpilib.SmartDashboard.putNumber("Elevator PowerOutput", percentageOutput)

        # wpimath.filter.SlewRateLimiter(3)
        self.motorFirst.set((percentageOutput + 0.14) * -0.75)
        self.motorSecond.set((percentageOutput + 0.14) * -0.75)

    def atGoal(self):
        return (self.encoder.getPosition() == self.goal) and (self.encoder.getVelocity() == 0)


class ElevatorToL1(commands2.Command):
    def __init__(self, elevator: Elevator):
        super().__init__()
        self._sub = elevator
        self.addRequirements(self._sub)

    def execute(self):
        self._sub._goToL1()
        if self._sub.atGoal():
            self.isFinished()


class ElevatorToL2(commands2.Command):
    def __init__(self, elevator: Elevator):
        super().__init__()
        self._sub = elevator
        self.addRequirements(self._sub)

    def execute(self):
        self._sub._goToL2()
        if self._sub.atGoal():
            self.isFinished()


class ElevatorToL3(commands2.Command):
    def __init__(self, elevator: Elevator):
        super().__init__()
        self._sub = elevator
        self.addRequirements(self._sub)

    def execute(self):
        self._sub._goToL3()
        if self._sub.atGoal():
            self.isFinished()


class ElevatorToL4(commands2.Command):
    def __init__(self, elevator: Elevator):
        super().__init__()
        self._sub = elevator
        self.addRequirements(self._sub)

    def execute(self):
        self._sub._goToL4()
        if self._sub.atGoal():
            self.isFinished()


class ElevatorToRest(commands2.Command):
    def __init__(self, elevator: Elevator):
        super().__init__()
        self._sub = elevator
        self.addRequirements(self._sub)

    def execute(self):
        self._sub._goToRest()
        if self._sub.atGoal():
            self.isFinished()
