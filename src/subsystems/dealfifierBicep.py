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
                self.constants.DABMaxVelocity,
                self.constants.DABMaxAccel,  # Max elevator speed and acceleration.
            )
        )

        self.PIDController = wpimath.controller.ProfiledPIDController(
            0.1, 0, 0, wpimath.trajectory.TrapezoidProfile.Constraints(
                self.constants.DABMaxVelocity,
                self.constants.DABMaxAccel
            )
        )

        self.motorFirst = rev.SparkMax(self.constants.DealgifierBicep, rev.SparkMax.MotorType.kBrushless)

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

        wpilib.SmartDashboard.putNumber("Bicep Goal", 0)

    def initMovement(self) -> None:

        # Reset our last reference to the current state.
        self.lastProfiledReference = wpimath.trajectory.TrapezoidProfile.State(
            self.encoder.getPosition(),
            wpimath.units.rotationsPerMinuteToRadiansPerSecond(self.encoder.getVelocity())
        )

    def disable(self):
        self.motorFirst.setVoltage(0)

    def periodic(self) -> None:
        # Sets the target position of our arm. This is similar to setting the setpoint of a
        # PID controller.

        percentageOutput = self.PIDController.calculate(
            self.encoder.getPosition(),
            wpilib.SmartDashboard.getNumber("Bicep Goal", 0)
        )

        wpilib.SmartDashboard.putNumber("Bicep Position", self.encoder.getPosition())
        wpilib.SmartDashboard.putNumber("Bicep PowerOutput", percentageOutput)

        # wpimath.filter.SlewRateLimiter(3)
        self.motorFirst.set((percentageOutput) * -0.75)

    def atGoal(self):
        return (self.encoder.getPosition() == self.goal) and (self.encoder.getVelocity() == 0)
