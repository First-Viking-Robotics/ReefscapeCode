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
from subsystems import network


class Elevator(commands2.Subsystem):
    def __init__(self, network: network.NetworkingAssistant, enabled: bool):
        super().__init__()
        self.autoElevateEnabled = False
        self.timer = wpilib.Timer()
        self.timer.reset()
        
        self.PIDController = wpimath.controller.ProfiledPIDController(
            0.6, 0, 0, wpimath.trajectory.TrapezoidProfile.Constraints(
                0.7,
                0.7
            )
        )
        self.PIDController.setTolerance(0.2, 0.2)

        # self.PIDController.setIntegratorRange(-0.1, 0.1)

        self.enabled = enabled
        self.net = network

        self.motorFirst = rev.SparkMax(26, rev.SparkMax.MotorType.kBrushless)
        self.motorSecond = rev.SparkMax(27, rev.SparkMax.MotorType.kBrushless)
        self.motorFirst.setVoltage(12)
        self.motorSecond.setVoltage(12)
        self.offset = 0

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
    
    def resetOffset(self):
        self.offset = 0
        self.autoElevateEnabled = False

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
    
    def increaseOffset(self):
        return commands2.cmd.run(
            lambda: self._increaseOffset(), self
        )
    
    def decreaseOffset(self):
        return commands2.cmd.run(
            lambda: self._decreaseOffset(), self
        )

    def periodic(self):
        return commands2.cmd.run(
            lambda: self._periodic(), self
            ).ignoringDisable(True)

    def _goToL1(self):
        self.resetOffset()
        self.goal = 0

    def _goToL2(self):
        self.resetOffset()
        self.goal = 0.3

    def _goToL3(self):
        self.resetOffset()
        self.goal = 1.7

    def _goToL4(self):
        self.resetOffset()
        self.goal = 4

    def _goToRest(self):
        self.resetOffset()
        self.goal = 0
    
    def _increaseOffset(self):
        self.offset = self.offset + 0.003
        self._periodic()
    
    def _decreaseOffset(self):
        self.offset = self.offset - 0.003
        self._periodic()

    def disable(self):
        self.motorFirst.getClosedLoopController().setReference(0, rev.SparkMax.ControlType.kDutyCycle)
        self.motorSecond.getClosedLoopController().setReference(0, rev.SparkMax.ControlType.kDutyCycle)

    def _periodic(self) -> None:
        # Sets the target position of our arm. This is similar to setting the setpoint of a
        # PID controller.

        percentageOutput = self.PIDController.calculate(
                self.encoder.getPosition(),
                (self.goal + self.offset)
            )
            

        # wpilib.SmartDashboard.putNumber("Elevator Position", self.encoder.getPosition())
        # wpilib.SmartDashboard.putNumber("Offset Position", self.offset)
        
        self.net.mainTable.putString("Elevator Temp", str((self.motorFirst.getMotorTemperature() * (9/5)) + 32) + " / " + str((self.motorSecond.getMotorTemperature() * (9/5)) + 32) + "deg F")
        

        # wpilib.SmartDashboard().putValue("Elevator Value", self.encoder.getPosition())

        # wpimath.filter.SlewRateLimiter(3)
        self.net.mainTable.putNumber("Elevator Pos", self.encoder.getPosition())
        if self.enabled:
            if self.goal < 0.08:
                self.motorFirst.getClosedLoopController().setReference(0, rev.SparkMax.ControlType.kDutyCycle)
                self.motorSecond.getClosedLoopController().setReference(0, rev.SparkMax.ControlType.kDutyCycle)
                self.net.mainTable.putNumber("Elevator Power", 0)
            else:
                self.motorFirst.getClosedLoopController().setReference((percentageOutput + 0.29) * -0.75, rev.SparkMax.ControlType.kDutyCycle)
                self.motorSecond.getClosedLoopController().setReference((percentageOutput + 0.29) * -0.75, rev.SparkMax.ControlType.kDutyCycle)
                self.net.mainTable.putNumber("Elevator Power", (percentageOutput + 0.29) * -0.75)
    
#     def waitForElevator(self, cmd: commands2.Command):
#         if self.autoElevateEnabled is False:
#             self.autoElevateEnabled = True
#             self.timer.reset()
#             self.timer.start()
        
#         if self.PIDController.atGoal() or self.timer.hasElapsed(5):
#             self.timer.stop()
#             cmd.isFinished = lambda: True
#         return cmd

# class Wait(commands2.Command):
#     def __init__(self, elevator: Elevator):
#         self.elevator = elevator
#         super().__init__()
    
#     def execute(self):
#         self.elevator.waitForElevator()
    