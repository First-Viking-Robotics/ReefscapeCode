import wpilib
import wpimath.filter
from pathplannerlib.auto import AutoBuilder

from src.subsystems import drivetrain


class RobotContainer:
    def __init__(self, constants):
        self.constants = constants
        self.controller = wpilib.XboxController(0)
        self.swerve = drivetrain.Drivetrain(self.constants)

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        # Example Register Named Commands
        # NamedCommands.registerCommand('autoBalance', self.swerve.autoBalanceCommand())
        # NamedCommands.registerCommand('exampleCommand', exampleSubsystem.exampleCommand())
        # NamedCommands.registerCommand('someOtherCommand', SomeOtherCommand())

        # Build an auto chooser. This will use Commands.none() as the default option.
        self.autoChooser = AutoBuilder.buildAutoChooser()

        # Another option that allows you to specify the default auto by its name
        self.autoChooser = AutoBuilder.buildAutoChooser("My Default Auto")

        wpilib.SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()

    def driveWithJoystick(self, getPeriod, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = (
                -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftY(), 0.02)
            )
                * self.constants.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = (
                -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftX(), 0.02)
            )
                * self.constants.kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
                -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRightX(), 0.02)
            )
                * self.constants.kMaxSpeed
        )

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod())
