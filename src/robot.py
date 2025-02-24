#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib.drive

from RobotContainer import RobotContainer


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.container = RobotContainer()

    def autonomousPeriodic(self) -> None:
        self.container.driveWithJoystick(self.getPeriod, False)
        self.container.swerve.updateOdometry()

    def teleopPeriodic(self) -> None:
        self.container.driveWithJoystick(self.getPeriod, False)

    def disabledInit(self) -> None:
        self.container.disable()
    
    def testPeriodic(self):
        pass
