#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math
import wpilib
import wpimath.filter
import wpimath.geometry
import wpimath.kinematics
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
import phoenix6
import wpimath.units

from subsystems import swervemodule
import constants
import commands2
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController

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
            turningEncoderChannel=self.constants.frontLeftTurningEncoderChannel,
            offset=-45
        )
        self.frontRight = swervemodule.SwerveModule(
            driveMotorChannel=self.constants.frontRightDriveMotorChannel,
            turningMotorChannel=self.constants.frontRightTurningMotorChannel,
            turningEncoderChannel=self.constants.frontRightTurningEncoderChannel,
            offset=45
        )
        self.backLeft = swervemodule.SwerveModule(
            driveMotorChannel=self.constants.backLeftDriveMotorChannel,
            turningMotorChannel=self.constants.backLeftTurningMotorChannel,
            turningEncoderChannel=self.constants.backLeftTurningEncoderChannel,
            offset=45
        )
        self.backRight = swervemodule.SwerveModule(
            driveMotorChannel=self.constants.backRightDriveMotorChannel,
            turningMotorChannel=self.constants.backRightTurningMotorChannel,
            turningEncoderChannel=self.constants.backRightTurningEncoderChannel,
            offset=-45
        )

        # navX MXP using SPI
        # self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
        self.gyro = phoenix6.hardware.pigeon2.Pigeon2(self.constants.pigeonChannel)

        self.gyro.set_yaw(0)

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(self.constants.swerveDriveKinematics, self.getRotation2d(),
                                                    (
                                                        self.frontLeft.getPosition(),
                                                        self.frontRight.getPosition(),
                                                        self.backLeft.getPosition(),
                                                        self.backRight.getPosition()
                                                    )
                                                )
        
        config = RobotConfig.fromGUISettings()
        
        AutoBuilder.configure(
            self.getPose,
            self.resetPose,
            self.getRobotRelativeSpeeds,
            lambda speeds, feedforwards: self.drive(speeds),
            PPHolonomicDriveController(
                PIDConstants(0.01, 0.0, 0.0),
                PIDConstants(0.01, 0.0, 0.0)
            ),
            config,
            self.shouldFlipPath,
            self
        )
    
    def shouldFlipPath(self):
        return wpilib.DriverStation.getAlliance()

    def disable(self):
        self.frontLeft.stop()
        self.frontRight.stop()
        self.backLeft.stop()
        self.backRight.stop()
    
    def getHeading(self):
        return math.remainder(self.gyro.get_yaw().value, 360)
    
    def getRotation2d(self):
        return wpimath.geometry.Rotation2d.fromDegrees(self.getHeading())
    
    def periodic(self):
        wpilib.SmartDashboard.putNumber("RobotHeading", self.getHeading())

    def setModuleStates(self, desiredStates: list[wpimath.kinematics.SwerveModuleState]):
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, 1)
        # if rotating:
        self.frontLeft.setDesiredState(desiredStates[0])
        self.frontRight.setDesiredState(desiredStates[1])
        self.backLeft.setDesiredState(desiredStates[2])
        self.backRight.setDesiredState(desiredStates[3])
        # else:
        #     self.frontLeft.setDesiredState(desiredStates[0], 1)
        #     self.frontRight.setDesiredState(desiredStates[1], 1)
        #     self.backLeft.setDesiredState(desiredStates[2], 1)
        #     self.backRight.setDesiredState(desiredStates[3], 1)
    
    def joystickDrive(self, xSpeed: float, ySpeed: float, rot: float, notFieldCentric: bool, forward: bool, slowMode: bool):
        FieldCentric = not notFieldCentric

        if slowMode:
            xSpeed = xSpeed * 0.1
            ySpeed = ySpeed * 0.1
        
        if FieldCentric:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, self.getRotation2d())
            moduleStates = self.constants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds)
            self.setModuleStates(moduleStates)
        else:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)
            moduleStates = self.constants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds)
            self.setModuleStates(moduleStates)
    
    def drive(self, chassisSpeeds: wpimath.kinematics.ChassisSpeeds):
        moduleStates = self.constants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds)
        self.setModuleStates(moduleStates)

    def resetPose(self, pose):
        return self.odometry.resetPose(pose)
    
    def getModuleStates(self):
        return (
                self.frontLeft.getState(),
                self.frontRight.getState(),
                self.backLeft.getState(),
                self.backRight.getState()
            )

    def getRobotRelativeSpeeds(self):
        return self.constants.swerveDriveKinematics.toChassisSpeeds(self.getModuleStates())
    
    def getPose(self):
        return self.odometry.getPose()
