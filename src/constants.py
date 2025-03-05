import math

import pathplannerlib.config
import wpimath.geometry
import wpimath.kinematics
import wpimath.units


class Constants:
    def __init__(self):
        # PID For autonomous movement
        self.translationalPIDConstants = pathplannerlib.config.PIDConstants(5.0, 0.0, 0.0)
        self.rotationalPIDConstants = pathplannerlib.config.PIDConstants(5.0, 0.0, 0.0)

        # Physical Drive Constants
        self.driveBaseRadius = 2
        self.maxModuleSpeed = 2
        self.kMaxSpeed = 1  # 1 meters per second
        self.kMaxAngularSpeed = 1.25
        self.kWheelRadius = 0.0508
        self.kEncoderResolution = 4096
        self.kModuleMaxAngularVelocity = math.pi
        self.kModuleMaxAngularAcceleration = math.tau
        globalDistance = 0.3143758
        self.frontLeftLocation = wpimath.geometry.Translation2d(globalDistance, globalDistance)
        self.frontRightLocation = wpimath.geometry.Translation2d(globalDistance, -globalDistance)
        self.backLeftLocation = wpimath.geometry.Translation2d(-globalDistance, globalDistance)
        self.backRightLocation = wpimath.geometry.Translation2d(-globalDistance, -globalDistance)
        self.swerveDriveKinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation
        )

        # Physical Elevator Constants
        self.ElevatorJoystickPort = 0

        # Juanita Buttons

        class JuanitaButtons:
            def __init__(self):
                self.L1 = 5
                self.L2 = 1
                self.L3 = 2
                self.L4 = 6
                self.Choral = 3  # Correct
                self.Algy = 4  # Correct

        self.JuanitaButtons = JuanitaButtons()

        # Elevator Goal Positions
        self.ElevatorRest = wpimath.units.feetToMeters(1.5)
        self.ElevatorL1 = wpimath.units.feetToMeters(1)
        self.ElevatorL2 = wpimath.units.feetToMeters(2)
        self.ElevatorL3 = wpimath.units.feetToMeters(3)
        self.ElevatorL4 = wpimath.units.feetToMeters(4)

        self.ElevatorMaxVelocity = wpimath.units.rotationsToRadians(0.5)
        self.ElevatorMaxAccel = wpimath.units.rotationsToRadians(0.25)

        self.ElevatorPositionErrorTolerance = wpimath.units.inchesToMeters(1.0)
        self.ElevatorVelocityErrorTolerance = wpimath.units.inchesToMeters(10.0)
        # qelms. Position and velocity error tolerances, in meters and meters per second. Decrease this to more
        # heavily penalize state excursion, or make the controller behave more aggressively. In
        # this example we weight position much more highly than velocity, but this can be
        # tuned to balance the two.

        self.ElevatorControlEffortTolerance = (12.0)
        # relms. Control effort (voltage) tolerance. Decrease this to more
        # heavily penalize control effort, or make the controller less aggressive. 12 is a good
        # starting point because that is the (approximate) maximum voltage of a battery.

        self.ElevatorNominalLoopTime = 0.020
        # Nominal time between loops. 0.020 for TimedRobot, but can be lower if using notifiers.

        self.ElevatorCarriageMass = 4.5  # In kilograms

        self.ElevatorDrumRadius = 1.106 / 2.0 * 25.4 / 1000.0  # A 1.5in diameter drum has a radius of 0.75in, or 0.019in.

        # Reduction between motors and encoder, as output over input. If the elevator spins slower than
        # the motors, this number should be greater than one.
        self.ElevatorGearing = 6.88

        # --- DealgifierBicep ---
        self.DABMaxVelocity = wpimath.units.rotationsToRadians(0.5)
        self.DABMaxAccel = wpimath.units.rotationsToRadians(0.25)

        # --- CAN Identification Numbers ---

        # Pigeon
        self.pigeonChannel = 5

        # Front Left Swerve Module
        self.frontLeftDriveMotorChannel = 10
        self.frontLeftTurningMotorChannel = 11
        self.frontLeftTurningEncoderChannel = 12

        # Front Right Swerve Module
        self.frontRightDriveMotorChannel = 14
        self.frontRightTurningMotorChannel = 15
        self.frontRightTurningEncoderChannel = 16

        # Back Left Swerve Module
        self.backLeftDriveMotorChannel = 18
        self.backLeftTurningMotorChannel = 19
        self.backLeftTurningEncoderChannel = 20

        # back Right Swerve Module
        self.backRightDriveMotorChannel = 22
        self.backRightTurningMotorChannel = 23
        self.backRightTurningEncoderChannel = 24

        # Elevator
        self.ElevatorFirstMotorPort = 26
        self.ElevatorSecondMotorPort = 27

        # Dealgifier
        self.DealgifierHandLeft = 29
        self.DealgifierHandRight = 30
        self.DealgifierBicep = 31

        # Choral Scorer
        self.ChoralScorerRight = 32
        self.ChoralScorerLeft = 33
