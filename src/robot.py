#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import RobotContainer
import commands2


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.robotContainer = RobotContainer.RobotContainer()

    def autonomousInit(self):
        self.robotContainer.runAuto()
        return super().autonomousInit()
