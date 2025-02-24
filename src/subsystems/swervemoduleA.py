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
import wpimath.units

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
        self.drivePIDController = wpimath.controller.PIDController(0.25, 0, 0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = wpimath.controller.PIDController(
            0.25,
            0,
            0
        )

        # self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def disable(self):
        self.driveMotor.set(0)
        self.turningMotor.set(0)

    def setDesiredState(
        self, angle, speed
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveMotor.get_velocity().value * self.constants.kWheelRadius * 2 * math.pi, speed
        )

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            wpimath.units.rotationsToRadians(self.driveMotor.get_position().value), angle
        )

        self.driveMotor.setVoltage(driveOutput)
        self.turningMotor.setVoltage(turnOutput)
