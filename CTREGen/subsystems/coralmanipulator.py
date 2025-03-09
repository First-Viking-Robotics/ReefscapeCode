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
    def __init__(self):
        self.leftMotor = rev.SparkMax(33, rev._rev.SparkLowLevel.MotorType.kBrushless)
        self.rightMotor = rev.SparkMax(32, rev._rev.SparkLowLevel.MotorType.kBrushless)
        self.leftEncoder = self.leftMotor.getEncoder()
        self.rightEncoder = self.rightMotor.getEncoder()
        self.beamBreakElevator = wpilib.DigitalInput(8)
        self.beamBreakReef = wpilib.DigitalInput(9)
        self.PIDLeft = controller.PIDController(0.5,0 ,0)
        self.PIDRight = controller.PIDController(0.5,0 ,0)
        self.speed = 0
        super().__init__()
    
    def _shooting(self):
        self.speed = 0.5
    
    def _holding(self):
        self.speed = 0
    
    def _sucking(self):
        self.speed = -0.5
    
    def _periodic(self):
        if self.beamBreakElevator.get():
            if self.speed == 0:
                self.leftMotor.getClosedLoopController().setReference(
                    self.PIDLeft.calculate(
                        self.leftEncoder.getPosition(),
                        0
                    ),
                    rev.SparkMax.ControlType.kDutyCycle
                )
                self.rightMotor.getClosedLoopController().setReference(
                    self.PIDRight.calculate(
                        self.rightEncoder.getPosition(),
                        0
                    ),
                    rev.SparkMax.ControlType.kDutyCycle
                )
                
            else:
                self.leftEncoder.setPosition(0)
                self.rightEncoder.setPosition(0)
                self.leftMotor.getClosedLoopController().setReference(-self.speed, rev.SparkMax.ControlType.kDutyCycle)
                self.rightMotor.getClosedLoopController().setReference(self.speed, rev.SparkMax.ControlType.kDutyCycle)
        else:
            self.leftEncoder.setPosition(0)
            self.rightEncoder.setPosition(0)
            self.leftMotor.getClosedLoopController().setReference(-0.1, rev.SparkMax.ControlType.kDutyCycle)
            self.rightMotor.getClosedLoopController().setReference(0.1, rev.SparkMax.ControlType.kDutyCycle)
        wpilib.SmartDashboard.putNumber("Coral Speed", self.speed)
        wpilib.SmartDashboard.putBoolean("Coral There Elevator", self.beamBreakElevator.get())
        wpilib.SmartDashboard.putBoolean("Coral There Reef", self.beamBreakReef.get())
        
    
    def periodic(self):
        return commands2.cmd.run(
            lambda: self._periodic(), self
            )
    
    def shooting(self):
        return commands2.cmd.runOnce(
            lambda: self._shooting(), self
            )
    
    def holding(self):
        return commands2.cmd.runOnce(
            lambda: self._holding(), self
            )
