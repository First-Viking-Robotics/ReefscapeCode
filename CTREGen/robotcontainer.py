#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians
from pathplannerlib.auto import AutoBuilder, NamedCommands
from wpilib import SmartDashboard
from subsystems import elevator, coralmanipulator


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.RobotCentric()
            .with_deadband(self._max_speed * 0.01)
            .with_rotational_deadband(
                self._max_angular_rate * 0.08
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)
        self._controlPanel = commands2.button.CommandGenericHID(1)

        self.drivetrain = TunerConstants.create_drivetrain()

        self.elevator = elevator.Elevator(enabled=True)

        self.coralManipulator = coralmanipulator.CoralScorer()

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("MainAuto")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)
        NamedCommands.registerCommand("shootCoral", self.coralManipulator.shooting())
        # NamedCommands.registerCommand("holdCoral", self.coralManipulator.holding())
        NamedCommands.registerCommand("L1", self.elevator.goToL1())
        NamedCommands.registerCommand("L2", self.elevator.goToL2())
        NamedCommands.registerCommand("L3", self.elevator.goToL3())
        NamedCommands.registerCommand("L4", self.elevator.goToL4())

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Set the Coral Scorer Default Command
        self.coralManipulator.setDefaultCommand(
            # The Coral Manipulator will execute this command periodically
            self.coralManipulator.periodic()
        )

        # Set the Elevator Default Command
        self.elevator.setDefaultCommand(
            # The elevator will execute this command periodically
            self.elevator.periodic()
        )

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self._joystick.getLeftY() * self._max_speed * 0.5
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self._joystick.getLeftX() * self._max_speed * 0.5
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._joystick.getRightX() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )
        # self._joystick.x().whileTrue(
        #     self.drivetrain.apply_request(
        #         lambda: self.drivetrain.playMusic()
        #     )
        # )

        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

        # define elevator bindings
        self._controlPanel.button(1).onTrue(  # L2
            self.elevator.goToL2()
        )

        self._controlPanel.button(2).onTrue(  # L3
            self.elevator.goToL3()
        )

        self._controlPanel.button(5).onTrue(  # L1
            self.elevator.goToL1()
        )

        self._controlPanel.button(6).onTrue(  # L4
            self.elevator.goToL4()
        )

        self._controlPanel.button(3).onTrue(  # Coral
            self.coralManipulator.shooting()
        )

        self._controlPanel.button(3).onFalse(  # Coral
            self.coralManipulator.holding()
        )

        self._controlPanel.povUp().whileTrue(
            self.elevator.increaseOffset()
        )

        self._controlPanel.povDown().whileTrue(
            self.elevator.decreaseOffset()
        )

        # self._controlPanel.button(4).onTrue(commands2.cmd.print_("Button 4"))  # Algy
        # self._controlPanel.button(7).onTrue(commands2.cmd.print_("Button 7"))
        # self._controlPanel.button(8).onTrue(commands2.cmd.print_("Button 8"))

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self._auto_chooser.getSelected()
