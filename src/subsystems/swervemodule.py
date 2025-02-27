#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import phoenix6.swerve
import wpilib
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory
import phoenix6
import wpimath.units

kWheelRadius = wpimath.units.inchesToMeters(2)
kMaxMetersPerSec = 1
kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau


class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        turningEncoderChannel: int
    ) -> None:
        self.driveMotor = phoenix6.hardware.TalonFX(driveMotorChannel)
        self.turningMotor = phoenix6.hardware.TalonFX(turningMotorChannel)

        self.turningEncoder = phoenix6.hardware.CANcoder(turningEncoderChannel)

        self.control = phoenix6.controls.DutyCycleOut(0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = wpimath.controller.PIDController(
            0.1,
            0,
            0
        )

        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

        self.resetEncoders()
    
    def getDrivePosition(self):
        return self.driveMotor.get_position().value * 2 * math.pi * kWheelRadius
    
    def getTurningPosition(self):
        return wpimath.units.rotationsToRadians(self.turningEncoder.get_position().value)
    
    def getDriveVelocity(self):
        return self.driveMotor.get_velocity().value * 2 * math.pi * kWheelRadius
    
    def getTurningVelocity(self):
        return wpimath.units.rotationsToRadians(self.turningEncoder.get_velocity().value)
    
    def getAbsoluteEncoderRad(self):
        return wpimath.units.rotationsToRadians(self.turningEncoder.get_absolute_position().value)
    
    def resetEncoders(self):
        self.driveMotor.set_position(0)
        self.turningEncoder.set_position(self.turningEncoder.get_absolute_position().value)

    def getState(self):
        return wpimath.kinematics.SwerveModuleState(
            self.getDriveVelocity(),
            wpimath.geometry.Rotation2d(self.getTurningPosition())
            )
    
    def setDesiredState(self, 
                        state: wpimath.kinematics.SwerveModuleState):
        if (abs(state.speed) < 0.001):
            self.stop()

        wpimath.kinematics.SwerveModuleState.optimize(state, self.getState().angle)
        self.driveMotor.set_control(self.control.with_output(state.speed / kMaxMetersPerSec))
        self.turningMotor.set_control(self.control.with_output(self.turningPIDController.calculate(self.getTurningPosition(), state.angle.radians())))
        wpilib.SmartDashboard.putString("Swerve[" + str(self.turningEncoder.device_id) + "] state", str(state))

    def stop(self):
        self.driveMotor.set_control(self.control.with_output(0))
        self.turningMotor.set_control(self.control.with_output(0))
