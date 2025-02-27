import wpilib
import wpilib.interfaces
import wpimath.filter
from pathplannerlib.auto import AutoBuilder, NamedCommands

from subsystems import drivetrain
# from subsystems import elevator
from constants import Constants
from subsystems import dealgifier
from subsystems import coralmanipulator
import commands2


class RobotContainer():
    def __init__(self, constants=Constants()):
        self.constants = constants
        self.controller = wpilib.interfaces.GenericHID(0)
        self.Juanita = wpilib.interfaces.GenericHID(1)

        self.swerve = drivetrain.Drivetrain()
        self.swerve.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.swerve.joystickDrive(self.controller.getRawAxis(0) * 0.25, self.controller.getRawAxis(4) * 0.25, -self.controller.getRawAxis(1) * 0.25),
                self.swerve)
        )

        self.dealgifier = dealgifier.Dealgifier(30, 29)
        self.dealgifier.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.dealgifier.handle(
                    self.Juanita.getRawButton(self.constants.JuanitaButtons.Algy)
                ),
                self.dealgifier
            )
        )

        self.choralscorer = coralmanipulator.CoralScorer(32, 33)
        self.choralscorer.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.choralscorer.handle(
                    self.Juanita.getRawButton(self.constants.JuanitaButtons.Choral)
                ),
                self.choralscorer
            )
        )
        # self.elevator = elevator.Elevator()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        # Example Register Named Commands
        # NamedCommands.registerCommand('autoBalance', self.swerve.autoBalanceCommand())
        # NamedCommands.registerCommand('ElevatorL1', self.elevator.goToL1())
        # NamedCommands.registerCommand('ElevatorL2', self.elevator.goToL2())
        # NamedCommands.registerCommand('ElevatorL3', self.elevator.goToL3())
        # NamedCommands.registerCommand('ElevatorL4', self.elevator.goToL4())
        # NamedCommands.registerCommand('ElevatorRest', self.elevator.goToRest())
        # NamedCommands.registerCommand('someOtherCommand', SomeOtherCommand())

        # Build an auto chooser. This will use Commands.none() as the default option.
        # self.autoChooser = AutoBuilder.buildAutoChooser()

        # Another option that allows you to specify the default auto by its name
        # self.autoChooser = AutoBuilder.buildAutoChooser("My Default Auto")

        # wpilib.SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def disable(self):
        # self.elevator.disable()
        self.swerve.disable()

    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()
