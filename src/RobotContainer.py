import wpilib
import wpilib.interfaces
import wpimath.filter
from pathplannerlib.auto import AutoBuilder, NamedCommands, PathPlannerAuto

from subsystems import drivetrain
from subsystems import elevator
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
                lambda: self.swerve.joystickDrive(
                    self.controller.getRawAxis(0) * self.constants.kMaxSpeed,
                    -self.controller.getRawAxis(1) * -self.constants.kMaxSpeed,
                    self.controller.getRawAxis(4) * self.constants.kMaxAngularSpeed,
                    not self.controller.getRawButton(1),
                    self.controller.getRawButton(2),
                    self.controller.getRawButton(6)),
                self.swerve)
        )

        # self.dealgifier = dealgifier.Dealgifier(30, 29)
        # self.dealgifier.setDefaultCommand(
        #     commands2.RunCommand(
        #         lambda: self.dealgifier.handle(
        #             self.Juanita.getRawButton(self.constants.JuanitaButtons.Algy)
        #         ),
        #         self.dealgifier
        #     )
        # )

        self.choralscorer = coralmanipulator.CoralScorer(32, 33)
        self.choralscorer.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.choralscorer.handle(
                    self.Juanita.getRawButton(self.constants.JuanitaButtons.Choral)
                ),
                self.choralscorer
            )
        )

        self.elevator = elevator.Elevator()
        self.elevator.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.elevator.periodic(),
                self.elevator
            )
        )

        # Example Register Named Commands
        # NamedCommands.registerCommand('autoBalance', self.swerve.autoBalanceCommand())
        # NamedCommands.registerCommand('ElevatorL1', self.elevator.goToL1())
        # NamedCommands.registerCommand('ElevatorL2', self.elevator.goToL2())
        # NamedCommands.registerCommand('ElevatorL3', self.elevator.goToL3())
        # NamedCommands.registerCommand('ElevatorL4', self.elevator.goToL4())
        # NamedCommands.registerCommand('ElevatorRest', self.elevator.goToRest())
        # NamedCommands.registerCommand('someOtherCommand', SomeOtherCommand())
        NamedCommands.registerCommand('SpitCoral', self.choralscorer.getSpitCoralCommand())  # self.choralscorer.getSpitCoralCommand()

        # Another option that allows you to specify the default auto by its name
        self.autoChooser = AutoBuilder.buildAutoChooser("Main Auto")

        wpilib.SmartDashboard.putData("Auto Chooser", self.autoChooser)
    
    def runAuto(self):
        self.getAutonomousCommand().schedule()

    def disable(self):
        # self.elevator.disable()
        self.swerve.disable()

    def getAutonomousCommand(self) -> commands2.Command:
        return self.autoChooser.getSelected()
