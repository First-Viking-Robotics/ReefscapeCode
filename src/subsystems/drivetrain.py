#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import wpilib
import wpimath.geometry
import wpimath.kinematics
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
import pathplannerlib.config

import swervemodule
from src.constants import Constants

class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self, constants) -> None:
        self.constants = constants
        self.frontLeftLocation = self.constants.frontLeftLocation
        self.frontRightLocation = self.constants.frontRightLocation
        self.backLeftLocation = self.constants.backLeftLocation
        self.backRightLocation = self.constants.backRightLocation

        self.frontLeft = swervemodule.SwerveModule(
            driveMotorChannel=self.constants.frontLeftDriveMotorChannel,
            turningMotorChannel=self.constants.frontLeftTurningMotorChannel,
            driveEncoderChannel=self.constants.frontLeftDriveEncoderChannel,
            turningEncoderChannel=self.constants.frontLeftTurningEncoderChannel,
            constants=self.constants
        )
        self.frontRight = swervemodule.SwerveModule(
            driveMotorChannel=self.constants.frontRightDriveMotorChannel,
            turningMotorChannel=self.constants.frontRightTurningMotorChannel,
            driveEncoderChannel=self.constants.frontRightDriveEncoderChannel,
            turningEncoderChannel=self.constants.frontRightTurningEncoderChannel,
            constants=self.constants
        )
        self.backLeft = swervemodule.SwerveModule(
            driveMotorChannel=self.constants.backLeftDriveMotorChannel,
            turningMotorChannel=self.constants.backLeftTurningMotorChannel,
            driveEncoderChannel=self.constants.backLeftDriveEncoderChannel,
            turningEncoderChannel=self.constants.backLeftTurningEncoderChannel,
            constants=self.constants
        )
        self.backRight = swervemodule.SwerveModule(
            driveMotorChannel=self.constants.backRightDriveMotorChannel,
            turningMotorChannel=self.constants.backRightTurningMotorChannel,
            driveEncoderChannel=self.constants.backRightDriveEncoderChannel,
            turningEncoderChannel=self.constants.backRightTurningEncoderChannel,
            constants=self.constants
        )

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
        config = pathplannerlib.config.HolonomicPathFollowerConfig(
            driveBaseRadius=self.constants.driveBaseRadius,
            maxModuleSpeed=self.constants.maxModuleSpeed,
            replanningConfig=pathplannerlib.config.ReplanningConfig(
                enableInitialReplanning=True,
                enableDynamicReplanning=False,
                dynamicReplanningTotalErrorThreshold=1.0,
                dynamicReplanningErrorSpikeThreshold=0.25
            ),
            rotationConstants=self.constants.rotationalPIDConstants,
            translationConstants=self.constants.translationalPIDConstants
        )

        # Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            self.odometry.getPose,  # Robot pose supplier
            self.odometry.resetPosition,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.driveRobotRelative(speeds),
            # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController(
                # PPHolonomicController is the built in path following controller for holonomic drive trains
                self.constants.translationalPIDConstants,  # Translation PID constants
                self.constants.rotationalPIDConstants  # Rotation PID constants
            ),
            config,  # The robot configuration
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self  # Reference to this subsystem to set requirements
        )

    def disable(self):
        self.frontLeft.disable()
        self.frontRight.disable()
        self.backLeft.disable()
        self.backRight.disable()

    def driveRobotRelative(self, speeds):
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(speeds, 0.02)
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, self.constants.kMaxSpeed
        )
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])


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
            swerveModuleStates, self.constants.kMaxSpeed
        )
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def getRobotRelativeSpeeds(self):
        return self.kinematics.toChassisSpeeds((self.frontLeft.getState(), self.frontRight.getState(), self.backLeft.getState(), self.backRight.getState()))

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