#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import commands2
import rev
import wpilib
from wpimath import controller

class CoralScorer(commands2.Subsystem):
    def __init__(self, getGamestateFunc):
        self.CI = getGamestateFunc
        self.leftMotor = rev.SparkMax(33, rev._rev.SparkLowLevel.MotorType.kBrushless)
        self.rightMotor = rev.SparkMax(32, rev._rev.SparkLowLevel.MotorType.kBrushless)
        self.leftEncoder = self.leftMotor.getEncoder()
        self.rightEncoder = self.rightMotor.getEncoder()
        self.beamBreakElevator = wpilib.DigitalInput(8)
        self.beamBreakReef = wpilib.DigitalInput(9)
        self.PIDLeft = controller.PIDController(0.25, 0, 0)
        self.PIDRight = controller.PIDController(0.25, 0, 0)
        self.gameState = 99
        self.gameStateNew = 99
        self.setpoint = 0
        self.leftEncoder.setPosition(0)
        self.rightEncoder.setPosition(0)
        super().__init__()
    
    def _shooting(self):
        self.setpoint = self.setpoint + 0.1
        self._periodic()
    
    def _shootingAuto(self):
        self.setpoint = self.setpoint + 3
        self._periodic()
    
    def _holding(self):
        self._periodic()
    
    def _sucking(self):
        self.setpoint = self.setpoint - 0.1
        self._periodic()
    
    def _periodic(self):
        # if self.gameState != self.CI():
        #     self.preventSpinout()
        #     self.gameState = self.CI()
        self.leftMotor.getClosedLoopController().setReference(
                    self.PIDLeft.calculate(
                        self.leftEncoder.getPosition(),
                        -self.setpoint
                    ),
                    rev.SparkMax.ControlType.kDutyCycle
                )
        self.rightMotor.getClosedLoopController().setReference(
                    self.PIDRight.calculate(
                        self.rightEncoder.getPosition(),
                        self.setpoint
                    ),
                    rev.SparkMax.ControlType.kDutyCycle
                )
        return ((-self.setpoint - 0.2) < self.leftEncoder.getPosition() < (-self.setpoint + 0.2)) and ((self.setpoint - 0.2) < self.rightEncoder.getPosition() < (self.setpoint + 0.2))
        # if self.beamBreakElevator.get():

        #     if self.speed == 0:
        #         self.leftMotor.getClosedLoopController().setReference(
        #             self.PIDLeft.calculate(
        #                 self.leftEncoder.getPosition(),
        #                 0
        #             ),
        #             rev.SparkMax.ControlType.kDutyCycle
        #         )
        #         self.rightMotor.getClosedLoopController().setReference(
        #             self.PIDRight.calculate(
        #                 self.rightEncoder.getPosition(),
        #                 0
        #             ),
        #             rev.SparkMax.ControlType.kDutyCycle
        #         )
                
        #     else:
        #         self.leftEncoder.setPosition(0)
        #         self.rightEncoder.setPosition(0)
        #         self.leftMotor.getClosedLoopController().setReference(-self.speed, rev.SparkMax.ControlType.kDutyCycle)
        #         self.rightMotor.getClosedLoopController().setReference(self.speed, rev.SparkMax.ControlType.kDutyCycle)
        # else:
        #     self.leftEncoder.setPosition(0)
        #     self.rightEncoder.setPosition(0)
        #     self.leftMotor.getClosedLoopController().setReference(-0.1, rev.SparkMax.ControlType.kDutyCycle)
        #     self.rightMotor.getClosedLoopController().setReference(0.1, rev.SparkMax.ControlType.kDutyCycle)
        # wpilib.SmartDashboard.putNumber("Coral Speed", self.speed)
        # wpilib.SmartDashboard.putBoolean("Coral There Elevator", self.beamBreakElevator.get())
        # wpilib.SmartDashboard.putBoolean("Coral There Reef", self.beamBreakReef.get())
    
    def preventSpinout(self):
        wpilib.reportWarning("Spinout Prevention")
        self.setpoint = 0
        self.leftEncoder.setPosition(0)
        self.rightEncoder.setPosition(0)
        self.leftMotor.getClosedLoopController().setReference(0, rev.SparkMax.ControlType.kDutyCycle)
        self.rightMotor.getClosedLoopController().setReference(0, rev.SparkMax.ControlType.kDutyCycle)
        
    
    def periodic(self):
        return commands2.cmd.run(
            lambda: self._periodic(), self
            )
    
    def shooting(self):
        return commands2.cmd.run(
            lambda: self._shooting(), self
            )
    
    def shootAuto(self):
        # return SpitCoral(self)
        return commands2.cmd.runOnce(
            lambda: self._shootingAuto(), self
            )
    
    # def holdAuto(self):
    #     return HoldCoral(self)
    
    def holding(self):
        return commands2.cmd.runOnce(
            lambda: self._holding(), self
            )

# class SpitCoral(commands2.Command):
#     def __init__(self, coralScorer: CoralScorer):
#         super().__init__()
#         self._sub = coralScorer
#         self.addRequirements(self._sub)

#     def execute(self):
#         self._sub.leftMotor.set(-0.1)
#         self._sub.rightMotor.set(0.1)

# class HoldCoral(commands2.Command):
#     def __init__(self, coralScorer: CoralScorer):
#         super().__init__()
#         self._sub = coralScorer
#         self.addRequirements(self._sub)

#     def execute(self):
#         self._sub.leftMotor.set(0)
#         self._sub.rightMotor.set(0)

# class AutoShootCommand(commands2.RunCommand):
#     def __init__(self, subsystem: CoralScorer):
#         self.subsystem = subsystem
#         self.addRequirements(self.subsystem)
    
#     def execute(self):
#         self.subsystem.setpoint = self.subsystem.setpoint + 2
#         self.subsystem.leftMotor.getClosedLoopController().setReference(
#                     self.subsystem.PIDLeft.calculate(
#                         self.subsystem.leftEncoder.getPosition(),
#                         -self.subsystem.setpoint
#                     ),
#                     rev.SparkMax.ControlType.kDutyCycle
#                 )
#         self.subsystem.rightMotor.getClosedLoopController().setReference(
#                     self.subsystem.PIDRight.calculate(
#                         self.subsystem.rightEncoder.getPosition(),
#                         self.subsystem.setpoint
#                     ),
#                     rev.SparkMax.ControlType.kDutyCycle
#                 )
    
#     def isFinished(self):
#         return ((-self.subsystem.setpoint - 0.2) < self.subsystem.leftEncoder.getPosition() < (-self.subsystem.setpoint + 0.2)) and ((self.subsystem.setpoint - 0.2) < self.subsystem.rightEncoder.getPosition() < (self.subsystem.setpoint + 0.2))
    
#     def end(self):
#         self.setpoint = 0
#         self.subsystem.leftEncoder.setPosition(0)
#         self.subsystem.rightEncoder.setPosition(0)
#         self.subsystem.leftMotor.getClosedLoopController().setReference(0, rev.SparkMax.ControlType.kDutyCycle)
#         self.subsystem.rightMotor.getClosedLoopController().setReference(0, rev.SparkMax.ControlType.kDutyCycle)
