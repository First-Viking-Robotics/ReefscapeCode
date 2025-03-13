#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
import wpilib
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry
from wpimath.filter import MedianFilter

from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians
from pathplannerlib.auto import AutoBuilder, NamedCommands
from pathplannerlib.commands import PPHolonomicDriveController
from wpilib import SmartDashboard
from subsystems import elevator, coralmanipulator, network
import wpimath
import math
import phoenix6


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self, robotpyInstance) -> None:
        self.gyro = phoenix6.hardware.Pigeon2(15)
        self.net = network.NetworkingAssistant()
        self.slowMode = False
        self.limelightFilterX = MedianFilter(5)
        self.limelightFilterY = MedianFilter(5)
        self.limelightFilterRot = MedianFilter(5)
        self.net.mainTable.putNumber("LimeLRVar", 1)
        self.net.mainTable.putNumber("LimeFBVar", 1)
        self.timer = wpilib.Timer()
        self.autoLineUpStarted = False
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

        self.mainInstance = robotpyInstance

        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)
        self._controlPanel = commands2.button.CommandGenericHID(1)

        self.drivetrain = TunerConstants.create_drivetrain()

        self.elevator = elevator.Elevator(network=self.net, enabled=True)


        self.coralManipulator = coralmanipulator.CoralScorer(getGamestateFunc=self.mainInstance.gameState.getGameState)

        self._driveTrainLimelightPIDForwardBackward = wpimath.controller.PIDController(
            0.2, 0, 0
        )
        self._driveTrainLimelightPIDLeftRight = wpimath.controller.PIDController(
            1, 0, 0
        )
        self._driveTrainLimelightPIDRot = wpimath.controller.PIDController(
            0.05, 0, 0
        )
        self._driveTrainLimelightPIDForwardBackward.setSetpoint(14)
        self._driveTrainLimelightPIDRot.setSetpoint(0)
        self._driveTrainLimelightPIDLeftRight.setSetpoint(0)

        self.autoCommand = self.drivetrain.apply_request(
            lambda: self.limelightAlign()
        )


        # Path follower
        NamedCommands.registerCommand("shootCoral", self.coralManipulator.shootAuto())
        NamedCommands.registerCommand("lineUp", self.autoCommand)
        # NamedCommands.registerCommand("holdCoral", self.coralManipulator.holdAuto())
        # NamedCommands.registerCommand("holdCoral", self.coralManipulator.holding())
        NamedCommands.registerCommand("L1", self.elevator.goToL1())
        NamedCommands.registerCommand("L2", self.elevator.goToL2())
        NamedCommands.registerCommand("L3", self.elevator.goToL3())
        NamedCommands.registerCommand("L4", self.elevator.goToL4())

        self._auto_chooser = AutoBuilder.buildAutoChooser("ShootTest")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

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
                lambda: self.driveJoy()
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

        self.limelightAlignCommand = LimelightAlign(self, 5, 20)

        self._joystick.x().onTrue(
            self.drivetrain.apply_request(
                lambda: self.limelightAlign()
            )
        )
        self._joystick.x().onFalse(
            self.drivetrain.apply_request(
                lambda: self.driveJoy()
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

        self._joystick.rightBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.slowModeEnable())
        )

        self._joystick.rightBumper().onFalse(
            self.drivetrain.runOnce(lambda: self.slowModeDisable())
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

        self._controlPanel.button(3).whileTrue(  # Coral
            self.coralManipulator.shooting()
        )

        self._controlPanel.button(4).whileTrue(  # Coral Slow
            self.coralManipulator.shootingSlow()
        )

        self._controlPanel.button(7).whileTrue( # Coral Suck
            self.coralManipulator.sucking()
        )

        self._controlPanel.povUp().whileTrue(
            self.elevator.increaseOffset()
        )

        self._controlPanel.povDown().whileTrue(
            self.elevator.decreaseOffset()
        )

        # self._controlPanel.button(8).onTrue(commands2.cmd.print_("Button 8"))
    
    def driveJoy(self):
        self.net.limelightTable.putNumber("ledMode", 1)
        self.net.mainTable.putNumber("Rot", self.gyro.get_yaw().value)
        if self.slowMode:
            xSpeed = -wpimath.applyDeadband(self._joystick.getLeftY(), 0.07) * 0.25
            ySpeed = -wpimath.applyDeadband(self._joystick.getLeftX(), 0.07) * 0.25
            rotSpeed = -wpimath.applyDeadband(self._joystick.getRightX(), 0.07) * 0.5
        else:
            xSpeed = -wpimath.applyDeadband(self._joystick.getLeftY(), 0.07)
            ySpeed = -wpimath.applyDeadband(self._joystick.getLeftX(), 0.07)
            rotSpeed = -wpimath.applyDeadband(self._joystick.getRightX(), 0.07)
        
        return self._drive.with_velocity_x(
                        xSpeed * self._max_speed * 0.5  # Drive forward with negative Y (forward)
                    ).with_velocity_y(
                        ySpeed * self._max_speed * 0.5  # Drive left with negative X (left)
                    ).with_rotational_rate(
                        rotSpeed * self._max_angular_rate  # Drive counterclockwise with negative X (left)
                    )
    
    def limelightAlign(self):
        if not self.autoLineUpStarted:
            self.autoLineUpStarted = True
            self.timer.reset()
            self.timer.start()
        self.net.limelightTable.putNumber("priorityid", 20)
        self.net.limelightTable.putNumber("ledMode", 3)
        if self.net.limelightTable.getNumber("tid", -8) == 20:
            forwardBackward = self.limelightFilterX.calculate(
                self.net.limelightTable.getNumber("ta", self._driveTrainLimelightPIDForwardBackward.getSetpoint())
            )
            rot = self.limelightFilterRot.calculate(
                self.net.limelightTable.getNumber("tx", self._driveTrainLimelightPIDRot.getSetpoint())
            )
            self.net.mainTable.putNumber("LimeRot", rot)
            self.net.mainTable.putNumber("LimeforwardBackward", forwardBackward)
            xSpeed = self._driveTrainLimelightPIDForwardBackward.calculate(
                measurement=forwardBackward
                )
            rotSpeed = self._driveTrainLimelightPIDRot.calculate(
                measurement=rot
                )
        else:
            xSpeed = 0
            rotSpeed = 0
        if self.timer.hasElapsed(3):
            self.timer.stop()
            self.autoCommand.isFinished = lambda: True
        return self._drive.with_velocity_x(
                        xSpeed * self._max_speed * 0.05  # Drive forward with negative Y (forward)
                    ).with_velocity_y(
                        rotSpeed * self._max_speed * 0.25  # Drive counterclockwise with negative X (left)
                    ).with_rotational_rate(
                        rotSpeed * self._max_angular_rate * 0.75  # Drive counterclockwise with negative X (left)
                    )
    
    def slowModeEnable(self):
        self.slowMode = True
    
    def slowModeDisable(self):
        self.slowMode = False
        

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self._auto_chooser.getSelected()


class LimelightAlign(commands2.Command):
    def __init__(self, robotContainer: RobotContainer, time: int, tagID: int):
        self.rc = robotContainer
        self.addRequirements(self.rc)
        self.timer = wpilib.Timer()
        self.time = time
        self.tagID = tagID
        self.command = self.rc.drivetrain.apply_request(
            lambda: self.rc.limelightAlign()
        )
    
    def initialize(self):
        self.timer.reset()
        self.timer.start()
        self.rc.net.limelightTable.putNumber("priorityid", self.tagID)
        self.command.schedule()
        # self.rc.net.limelightTable.putNumber("ledMode", 3)
    
    # def execute(self):
        # if self.rc.net.limelightTable.getNumber("tid", -8) == 20:
        #     forwardBackward = self.rc.limelightFilterX.calculate(
        #         self.rc.net.limelightTable.getNumber("ta", self.rc._driveTrainLimelightPIDForwardBackward.getSetpoint())
        #     )
        #     rot = self.rc.limelightFilterRot.calculate(
        #         self.rc.net.limelightTable.getNumber("tx", self.rc._driveTrainLimelightPIDRot.getSetpoint())
        #     )
        #     self.rc.net.mainTable.putNumber("LimeRot", rot)
        #     self.rc.net.mainTable.putNumber("LimeforwardBackward", forwardBackward)
        #     xSpeed = self.rc._driveTrainLimelightPIDForwardBackward.calculate(
        #         measurement=forwardBackward
        #         )
        #     # rotSpeed = self.rc._driveTrainLimelightPIDRot.calculate(
        #     #     measurement=rot
        #     #     )
        #     ySpeed = self.rc._driveTrainLimelightPIDLeftRight.calculate(
        #         measurement=rot
        #     )
        #     if ySpeed > 0.1:
        #         xSpeed = 0
        # else:
        #     ySpeed = 0
        #     xSpeed = 0
        
        # fieldCentricChassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
        #     vy=ySpeed,
        #     vx=xSpeed,
        #     omega=0,
        #     robotAngle=wpimath.geometry.Rotation2d.fromDegrees(self.rc.gyro.get_yaw().value)
        # )

        # PPHolonomicDriveController.overrideXFeedback(lambda: fieldCentricChassisSpeeds.vx)
        # PPHolonomicDriveController.overrideYFeedback(lambda: fieldCentricChassisSpeeds.vy)
        # PPHolonomicDriveController.overrideRotationFeedback(lambda: 0.0)
        # def rar():
        #     return 2
        # def rarb():
        #     return 0
        # PPHolonomicDriveController.overrideXFeedback(rarb)
        # PPHolonomicDriveController.overrideYFeedback(rarb)
        # PPHolonomicDriveController.overrideRotationFeedback(rar)

        # return self.rc.drivetrain.apply_request(
        #     self.rc._drive.with_velocity_x(
        #                 xSpeed * self.rc._max_speed * 0.05 * self.rc.net.mainTable.getNumber("LimeFBVar", 1)  # Drive forward with negative Y (forward)
        #             ).with_velocity_y(
        #                 ySpeed * self.rc._max_speed * 0.5 * self.rc.net.mainTable.getNumber("LimeLRVar", 1) # Drive left with negative X (left)
        #             ).with_rotational_rate(
        #                 0  # Drive counterclockwise with negative X (left)
        #             )
        # )
    
    def isFinished(self):
        return self.timer.hasElapsed(self.time)

    
    def end(self, interrupted):
        self.command.cancel()
        self.rc.net.limelightTable.putNumber("ledMode", 1)
        return super().end(interrupted)
    