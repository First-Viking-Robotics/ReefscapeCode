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
        turningEncoderChannel: int,
        offset: wpimath.units.degrees
    ) -> None:
        self.offset = wpimath.units.degreesToRadians(offset)
        self.driveMotor = phoenix6.hardware.TalonFX(driveMotorChannel)
        self.turningMotor = phoenix6.hardware.TalonFX(turningMotorChannel)

        self.turningEncoder = phoenix6.hardware.CANcoder(turningEncoderChannel)

        self.control = phoenix6.controls.DutyCycleOut(0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = wpimath.controller.PIDController(
            0.05,
            0,
            0
        )

        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

        self.resetEncoders()
    
    def getPosition(self):
        return wpimath.kinematics.SwerveModulePosition(self.getDrivePosition(), wpimath.geometry.Rotation2d.fromDegrees(wpimath.units.radiansToDegrees(self.getTurningPosition())))
    
    def getDrivePosition(self):
        return self.driveMotor.get_position().value * 2 * math.pi * kWheelRadius
    
    def getTurningPosition(self) -> wpimath.units.radians:
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
    
    def optimize(self, currentAngle: float, goalAngle: float):
        oneUp = goalAngle + 180
        oneDown = goalAngle - 180
        lst = [oneUp, goalAngle, oneDown]
        return lst[min(range(len(lst)), key=lambda i: abs(lst[i]-currentAngle))]
    
    def setDesiredState(self, state: wpimath.kinematics.SwerveModuleState):
        if (abs(state.speed) < 0.02):
            self.stop()

        currentAngle = self.getState().angle

        wpimath.kinematics.SwerveModuleState.optimize(state, currentAngle)

        goalAngle = self.optimize(currentAngle.radians(), state.angle.radians())
        # goalState = wpimath.kinematics.SwerveModuleState(goalSpeed, wpimath.geometry.Rotation2d.fromDegrees(wpimath.units.radiansToDegrees(goalAngle)))

        self.turningMotor.set_control(self.control.with_output(self.turningPIDController.calculate(self.getTurningPosition(), goalAngle)))  #  + self.offset
        self.driveMotor.set_control(self.control.with_output(state.speed))

        wpilib.SmartDashboard.putNumber("Swerve[" + str(self.turningEncoder.device_id) + "] GoalAngle", wpimath.units.radiansToDegrees(goalAngle))

    def stop(self):
        self.driveMotor.set_control(self.control.with_output(0))
        self.turningMotor.set_control(self.control.with_output(0))
