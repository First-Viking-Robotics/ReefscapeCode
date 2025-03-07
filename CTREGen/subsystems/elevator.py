import math

import commands2
import rev
import wpilib
import wpimath.controller
import wpimath.estimator
import wpimath.units
import wpimath.trajectory
import wpimath.system
import wpimath.system.plant
import phoenix6
import numpy as np


class Elevator(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.motorFirst = rev.CANSparkMax(self.constants.ElevatorFirstMotorPort, rev.CANSparkMax.MotorType.kBrushless)
        self.motorSecond = rev.CANSparkMax(self.constants.ElevatorSecondMotorPort, rev.CANSparkMax.MotorType.kBrushless)

        # A joystick to read the trigger from.
        self.joystick = wpilib.Joystick(self.constants.ElevatorJoystickPort)

        # Circumference = pi * d, so distance per click = pi * d / counts
        self.distancePerRotation = math.tau * self.constants.ElevatorDrumRadius

        # Preset Goal
        self.goal = self.constants.ElevatorRest

    def initMovement(self) -> None:
        # Reset our loop to make sure it's in a known state.
        self.loop.reset(np.array([
            (self.encoder.get_position().value * self.distancePerRotation),
            (self.encoder.get_velocity().value * self.distancePerRotation)
        ]))

        # Reset our last reference to the current state.
        self.lastProfiledReference = wpimath.trajectory.TrapezoidProfile.State(
            (self.encoder.get_position().value * self.distancePerRotation),
            (self.encoder.get_velocity().value * self.distancePerRotation)
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

        goal = wpimath.trajectory.TrapezoidProfile.State(self.goal, 0.0)

        # Step our TrapezoidalProfile forward 20ms and set it as our next reference
        self.lastProfiledReference = self.profile.calculate(
            0.020, self.lastProfiledReference, goal
        )
        self.loop.setNextR(
            np.array([self.lastProfiledReference.position, self.lastProfiledReference.velocity])
        )

        # Correct our Kalman filter's state vector estimate with encoder data.
        self.loop.correct(np.array([(self.encoder.get_position().value * self.distancePerRotation)]))

        # Update our LQR to generate new voltage commands and use the voltages to predict the next
        # state with out Kalman filter.
        self.loop.predict(0.020)

        # Send the new calculated voltage to the motors.
        # voltage = duty cycle * battery voltage, so
        # duty cycle = voltage / battery voltage
        nextVoltage = self.loop.U(0)
        self.motorFirst.setVoltage(nextVoltage)
        self.motorSecond.setVoltage(nextVoltage)

    def atGoal(self):
        return ((self.encoder.get_position().value * self.distancePerRotation) == self.goal) and ((self.encoder.get_velocity().value * self.distancePerRotation) == 0)


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
