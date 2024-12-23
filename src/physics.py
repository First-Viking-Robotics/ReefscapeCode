#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import phoenix6.sim.talon_fx_sim_state
#
# See the notes for the other physics sample
#

import wpilib.simulation
import phoenix5

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import drivetrains

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        """

        self.physics_controller = physics_controller

        # Motors
        self.lf_motor = phoenix5.TalonFX(0)
        self.lr_motor = phoenix5.TalonFX(1)
        self.rf_motor = phoenix5.TalonFX(2)
        self.rr_motor = phoenix5.TalonFX(3)
        self.lf_angle = phoenix6.hardware.CANcoder(4)
        self.lr_angle = phoenix6.hardware.CANcoder(5)
        self.rf_angle = phoenix6.hardware.CANcoder(6)
        self.rr_angle = phoenix6.hardware.CANcoder(7)

        # Gyro
        self.gyro = wpilib.simulation.AnalogGyroSim(robot.gyro)

        self.drivetrain = drivetrains.four_motor_swerve_drivetrain(
            self.lr_motor.getMotorOutputPercent(),
            self.rr_motor.getMotorOutputPercent(),
            self.lf_motor.getMotorOutputPercent(),
            self.lr_motor.getMotorOutputPercent(),
            self.lr_angle.get_absolute_position().value,
            self.rr_angle.get_absolute_position().value,
            self.lf_angle.get_absolute_position().value,
            self.lr_angle.get_absolute_position().value,
            1.25,
            1.25
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        lf_motor = self.lf_motor.getSpeed()
        lr_motor = self.lr_motor.getSpeed()
        rf_motor = self.rf_motor.getSpeed()
        rr_motor = self.rr_motor.getSpeed()

        speeds = self.drivetrain.calculate(lf_motor, lr_motor, rf_motor, rr_motor)
        pose = self.physics_controller.drive(speeds, tm_diff)

        self.gyro.setAngle(-pose.rotation().degrees())
