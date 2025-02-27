#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import commands2
import rev

class CoralScorer(commands2.Subsystem):
    def __init__(self, rightMotorChannel: int, leftMotorChannel: int):
        self.leftMotor = rev.SparkMax(leftMotorChannel, rev._rev.SparkLowLevel.MotorType.kBrushless)
        self.rightMotor = rev.SparkMax(rightMotorChannel, rev._rev.SparkLowLevel.MotorType.kBrushless)
        super().__init__()
    
    def holding(self):
        self.leftMotor.set(0.1)
        self.rightMotor.set(-0.1)

    def shoot(self):
        self.leftMotor.set(-0.1)
        self.rightMotor.set(0.1)
    
    def stop(self):
        self.leftMotor.set(0)
        self.rightMotor.set(0)
    
    def handle(self, buttonInput: bool):
        if buttonInput:
            self.shoot()
        else:
            self.stop()
