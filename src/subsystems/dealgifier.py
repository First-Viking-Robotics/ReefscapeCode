#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import commands2
import wpilib
import rev
import wpimath
import wpimath.units

class Dealgifier(commands2.Subsystem):
    def __init__(self, rightMotorChannel: int, leftMotorChannel: int):
        self.leftMotor = rev.SparkMax(leftMotorChannel, rev._rev.SparkLowLevel.MotorType.kBrushless)
        self.rightMotor = rev.SparkMax(rightMotorChannel, rev._rev.SparkLowLevel.MotorType.kBrushless)
        self.holdingToggle = False
        self.shooting = False
        self.shooterTimer = wpilib.Timer()
        super().__init__()
    
    def holding(self):
        # self.leftMotor.configure(rev.SparkMax.IdleMode.kBrake)
        # self.rightMotor.configure(rev.SparkMax.IdleMode.kBrake)
        self.leftMotor.set(-0.1)
        self.rightMotor.set(0.1)

    def shoot(self):
        # self.leftMotor.configure(rev.SparkMax.IdleMode.kCoast)
        # self.rightMotor.configure(rev.SparkMax.IdleMode.kCoast)
        self.leftMotor.set(0.5)
        self.rightMotor.set(-0.5)
    
    def stop(self):
        # self.leftMotor.configure(rev.SparkMax.IdleMode.kBrake)
        # self.rightMotor.configure(rev.SparkMax.IdleMode.kBrake)
        self.leftMotor.set(0)
        self.rightMotor.set(0)
    
    def handle(self, buttonInput: bool):
        if buttonInput:
            self.holdingToggle = True
            self.shooting = False
            self.holding()
            self.shooterTimer.stop()
        elif not buttonInput and self.holdingToggle:
            self.holdingToggle = False
            self.shooting = True
            self.shooterTimer.reset()
            self.shooterTimer.start()
        
        if wpimath.units.secondsToMilliseconds(self.shooterTimer.get()) > 1000:
            self.shooterTimer.stop()
            self.shooting = False

        if self.shooting and not buttonInput:
            self.shoot()
