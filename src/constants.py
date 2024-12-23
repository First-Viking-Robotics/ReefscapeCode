import math

import pathplannerlib.config
import wpimath.geometry


class Constants:
    def __int__(self):
        # PID For autonomous movement
        self.translationalPIDConstants = pathplannerlib.config.PIDConstants(5.0, 0.0, 0.0)
        self.rotationalPIDConstants = pathplannerlib.config.PIDConstants(5.0, 0.0, 0.0)

        # Physical Constants
        self.kMaxSpeed = 3.0  # 3 meters per second
        self.kMaxAngularSpeed = math.pi  # 1/2 rotation per second
        self.kWheelRadius = 0.0508
        self.kEncoderResolution = 4096
        self.kModuleMaxAngularVelocity = math.pi
        self.kModuleMaxAngularAcceleration = math.tau
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.381, 0.381)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.381, -0.381)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.381, 0.381)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.381, -0.381)

        # --- Electrical Ports ---

        # Front Left Swerve Module
        self.frontLeftDriveMotorChannel = 10
        self.frontLeftTurningMotorChannel = 11
        self.frontLeftDriveEncoderChannel = 12
        self.frontLeftTurningEncoderChannel = 13

        # Front Right Swerve Module
        self.frontRightDriveMotorChannel = 14
        self.frontRightTurningMotorChannel = 15
        self.frontRightDriveEncoderChannel = 16
        self.frontRightTurningEncoderChannel = 17

        # Back Left Swerve Module
        self.backLeftDriveMotorChannel = 18
        self.backLeftTurningMotorChannel = 19
        self.backLeftDriveEncoderChannel = 20
        self.backLeftTurningEncoderChannel = 21

        # back Right Swerve Module
        self.backRightDriveMotorChannel = 22
        self.backRightTurningMotorChannel = 23
        self.backRightDriveEncoderChannel = 24
        self.backRightTurningEncoderChannel = 25

