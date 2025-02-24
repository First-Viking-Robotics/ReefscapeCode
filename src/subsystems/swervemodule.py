#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import phoenix5
import phoenix6.hardware
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory

from constants import Constants


class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        turningEncoderChannel: int,
            constants: Constants
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      CAN bus channel for the drive motor.
        :param turningMotorChannel:    CAN bus channel for the turning motor.
        :param driveEncoderChannel:    CAN bus channel for the drive encoder channel
        :param turningEncoderChannel:  CAN bus channel for the turning encoder channel
        """
        self.constants = constants
        self.driveMotor = phoenix6.hardware.TalonFX(driveMotorChannel)
        self.turningMotor = phoenix6.hardware.TalonFX(turningMotorChannel)

        self.turningEncoder = phoenix6.hardware.CANcoder(turningEncoderChannel)

        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = wpimath.controller.PIDController(0.00000000001, 0, 0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            0.00000000001,
            0,
            0,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                self.constants.kModuleMaxAngularVelocity,
                self.constants.kModuleMaxAngularAcceleration,
            ),
        )

        # Gains are for example purposes only - must be determined for your own robot!
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0, 0)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0, 0)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def disable(self):
        self.driveMotor.set(phoenix5.ControlMode.Disabled)
        self.turningMotor.set(phoenix5.ControlMode.Disabled)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        # TODO: Make it get correct velocity and position
        return wpimath.kinematics.SwerveModuleState(
            self.driveMotor.get_velocity().value,
            wpimath.geometry.Rotation2d(self.turningEncoder.get_absolute_position().value),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveMotor.get_position().value * self.constants.kWheelRadius * 2 * math.pi,
            wpimath.geometry.Rotation2d(self.turningEncoder.get_absolute_position().value),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        encoderRotation = wpimath.geometry.Rotation2d(self.turningEncoder.get_absolute_position().value)

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoderRotation).cos()

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            state.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.getDistance(), state.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)
