#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
import navx
import wpimath.units

from subsystems import swervemodule
import constants
import commands2

class Drivetrain(commands2.Subsystem):
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self, constants=constants.Constants()) -> None:
        super().__init__()
        self.constants = constants
        self.frontLeftLocation = self.constants.frontLeftLocation
        self.frontRightLocation = self.constants.frontRightLocation
        self.backLeftLocation = self.constants.backLeftLocation
        self.backRightLocation = self.constants.backRightLocation

        self.frontLeft = swervemodule.SwerveModule(
            driveMotorChannel=self.constants.frontLeftDriveMotorChannel,
            turningMotorChannel=self.constants.frontLeftTurningMotorChannel,
            turningEncoderChannel=self.constants.frontLeftTurningEncoderChannel
        )
        self.frontRight = swervemodule.SwerveModule(
            driveMotorChannel=self.constants.frontRightDriveMotorChannel,
            turningMotorChannel=self.constants.frontRightTurningMotorChannel,
            turningEncoderChannel=self.constants.frontRightTurningEncoderChannel
        )
        self.backLeft = swervemodule.SwerveModule(
            driveMotorChannel=self.constants.backLeftDriveMotorChannel,
            turningMotorChannel=self.constants.backLeftTurningMotorChannel,
            turningEncoderChannel=self.constants.backLeftTurningEncoderChannel
        )
        self.backRight = swervemodule.SwerveModule(
            driveMotorChannel=self.constants.backRightDriveMotorChannel,
            turningMotorChannel=self.constants.backRightTurningMotorChannel,
            turningEncoderChannel=self.constants.backRightTurningEncoderChannel
        )

        # navX MXP using SPI
        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

        self.gyro.reset()

    def disable(self):
        self.frontLeft.stop()
        self.frontRight.stop()
        self.backLeft.stop()
        self.backRight.stop()
    
    def getHeading(self):
        return math.remainder(self.gyro.getAngle(), 360)
    
    def getRotation2d(self):
        return wpimath.geometry.Rotation2d.fromDegrees(self.getHeading())
    
    def periodic(self):
        wpilib.SmartDashboard.putNumber("RobotHeading", self.getHeading())

    def setModuleStates(self, desiredStates: list[wpimath.kinematics.SwerveModuleState]):
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, 1)
        self.frontLeft.setDesiredState(desiredStates[0])
        self.frontRight.setDesiredState(desiredStates[1])
        self.backLeft.setDesiredState(desiredStates[2])
        self.backRight.setDesiredState(desiredStates[3])
    
    def joystickDrive(self, xSpeed, ySpeed, rot):
        # self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        # self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        # self.rotLimiter = wpimath.filter.SlewRateLimiter(3)
        # xSpeed = (
        #         -self.xspeedLimiter.calculate(
        #         wpimath.applyDeadband(ySpeed, 0.02)
        #     )
        #         * 1
        # )
        # ySpeed = (
        #         -self.yspeedLimiter.calculate(
        #         wpimath.applyDeadband(xSpeed, 0.02)
        #     )
        #         * 1
        # )
        # rot = (
        #         -self.rotLimiter.calculate(
        #         wpimath.applyDeadband(rot, 0.02)
        #     )
        #         * 1
        # )
        if (False == True):
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, self.swerveSubsystem.getRotation2d()
            )
        else:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)
        moduleStates = self.constants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds)
        self.setModuleStates(moduleStates)
