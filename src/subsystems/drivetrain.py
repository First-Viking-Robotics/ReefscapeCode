#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import pathplannerlib.config
import wpilib
import wpimath.geometry
import wpimath.kinematics
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig

import swervemodule

kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second


class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.381, 0.381)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.381, -0.381)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.381, 0.381)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.381, -0.381)

        self.frontLeft = swervemodule.SwerveModule(1, 2, 0, 1, 2, 3)
        self.frontRight = swervemodule.SwerveModule(3, 4, 4, 5, 6, 7)
        self.backLeft = swervemodule.SwerveModule(5, 6, 8, 9, 10, 11)
        self.backRight = swervemodule.SwerveModule(7, 8, 12, 13, 14, 15)

        self.gyro = wpilib.AnalogGyro(0)

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        self.gyro.reset()

        # Load the RobotConfig from the GUI settings. You should probably
        # store this in your Constants file
        config = RobotConfig.fromGUISettings()

        # Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            self.odometry.getPose,  # Robot pose supplier
            self.odometry.resetPosition,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.driveRobotRelative(speeds),
            # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController(
                # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(5.0, 0.0, 0.0),  # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0)  # Rotation PID constants
            ),
            config,  # The robot configuration
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self  # Reference to this subsystem to set requirements
        )

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                (
                    wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                    )
                    if fieldRelative
                    else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)
                ),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def getRobotRelativeSpeeds(self):
        return self.kinematics.toChassisSpeeds([self.frontLeft.getState(), self.frontRight.getState(), self.backLeft.getState(), self.backRight.getState()])

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )