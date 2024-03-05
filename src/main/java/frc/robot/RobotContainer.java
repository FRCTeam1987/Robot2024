// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.control.AimLockWrist;
import frc.robot.commands.control.GoHome;
import frc.robot.commands.control.IdleShooter;
import frc.robot.commands.control.IntakeNoteSequence;
import frc.robot.commands.control.MoveGates;
import frc.robot.commands.control.PoopNote;
import frc.robot.commands.control.ReverseIntake;
import frc.robot.commands.control.ShootAmp;
import frc.robot.commands.control.ShootNote;
import frc.robot.commands.control.ShootNoteSequence;
import frc.robot.commands.control.SpitNote;
import frc.robot.commands.movement.CollectNoteAuto;
import frc.robot.commands.movement.DriveToNote;
import frc.robot.commands.movement.DriveToNoteAuto;
import frc.robot.commands.movement.PointAtAprilTag;
import frc.robot.commands.movement.ShootTrap;
import frc.robot.commands.movement.SquareUpToAprilTag;
import frc.robot.commands.movement.TeleopSwerve;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.candle.Candles;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

public class RobotContainer {
  private static RobotContainer instance;
  public final ShuffleboardTab COMMANDS_TAB = Shuffleboard.getTab("COMMANDS");
  public final ShuffleboardTab MATCH_TAB = Shuffleboard.getTab("MATCH");
  public static GenericEntry SHOOT_ANGLE;
  public static GenericEntry POOP_RPM;
  public final ShuffleboardTab PHOTON_TAB = Shuffleboard.getTab("PHOTON");
  public final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("SHOOTER");

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(0);
  private final CommandXboxController CO_DRIVER_CONTROLLER = new CommandXboxController(1);

  public final Drivetrain DRIVETRAIN = DriveConstants.DriveTrain; // My drivetrain

  public final Candles CANDLES = new Candles(Constants.LEFT_CANDLE, Constants.RIGHT_CANDLE);

  public final Intake INTAKE = new Intake(Constants.INTAKE_TOP_ID, Constants.INTAKE_BOTTOM_ID);
  public final Climber CLIMBER = new Climber(Constants.CLIMB_LEFT, Constants.CLIMB_RIGHT);
  public final Shooter SHOOTER =
      new Shooter(
          Constants.SHOOTER_LEADER_ID, Constants.SHOOTER_FOLLOWER_ID, Constants.SHOOTER_FEEDER_ID);
  public final Wrist WRIST = new Wrist(Constants.WRIST_ID);
  public final Elevator ELEVATOR =
      new Elevator(Constants.ELEVATOR_LEADER_ID, Constants.ELEVATOR_FOLLOWER_ID);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.MaxSpeed * 0.1)
          .withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(Constants.MaxSpeed);
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final SlewRateLimiter translationXSlewRate =
      new SlewRateLimiter(Constants.translationXSlewRate);
  private final SlewRateLimiter translationYSlewRate =
      new SlewRateLimiter(Constants.translationYSlewRate);
  private final SlewRateLimiter rotationSlewRate = new SlewRateLimiter(Constants.rotationSlewRate);

  private void configureBindings() {
    // DRIVETRAIN.setDefaultCommand( // Drivetrain will execute this command periodically
    //     DRIVETRAIN.applyRequest(
    //         () ->
    //             drive
    //                 .withVelocityX(
    //                     translationXSlewRate.calculate(DRIVER_CONTROLLER.getLeftY())
    //                         * Constants.MaxSpeed) // Drive forward with
    //                 // negative Y (forward)
    //                 .withVelocityY(
    //                     translationYSlewRate.calculate(DRIVER_CONTROLLER.getLeftX())
    //                         * Constants.MaxSpeed) // Drive left with negative X (left)
    //                 .withRotationalRate(
    //                     rotationSlewRate.calculate(-DRIVER_CONTROLLER.getRightX())
    //                         * Constants
    //                             .MaxAngularRate) // Drive counterclockwise with negative X (left)
    //         ));
    DRIVETRAIN.setDefaultCommand(
        new TeleopSwerve(
            DRIVETRAIN,
            () -> -DRIVER_CONTROLLER.getLeftY(), // left right
            () -> -DRIVER_CONTROLLER.getLeftX(), // Forward Backward
            () -> DRIVER_CONTROLLER.getRightX(),
            () -> 1.0,
            () -> DRIVER_CONTROLLER.getHID().getPOV(),
            () -> DRIVER_CONTROLLER.a().getAsBoolean()));

    // DRIVER_CONTROLLER.a().whileTrue(DRIVETRAIN.applyRequest(() -> brake));
    DRIVER_CONTROLLER
        .b()
        .whileTrue(
            DRIVETRAIN.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -DRIVER_CONTROLLER.getLeftY(), -DRIVER_CONTROLLER.getLeftX()))));

    DRIVER_CONTROLLER
        .back()
        .onTrue(
            new InstantCommand(
                () -> {
                  DRIVETRAIN.resetPose(new Pose2d());
                  System.out.println(DRIVETRAIN.getPose().getRotation());
                },
                DRIVETRAIN));
    DRIVER_CONTROLLER.start().onTrue(new GoHome(ELEVATOR, WRIST, SHOOTER, INTAKE));

    DRIVER_CONTROLLER
        .rightTrigger()
        .whileTrue(
            new PointAtAprilTag(
                DRIVETRAIN,
                Constants.SPEAKER_PROTON,
                () -> (DRIVER_CONTROLLER.getLeftX() * Constants.MaxSpeed),
                () -> (DRIVER_CONTROLLER.getLeftY() * Constants.MaxSpeed),
                () -> (DRIVER_CONTROLLER.getRightX() * Constants.MaxSpeed)));
    SHOOT_ANGLE = COMMANDS_TAB.add("Shoot Angle", 30).getEntry();
    DRIVER_CONTROLLER.x().onTrue(new PoopNote(SHOOTER, 500));
    // driverController
    //     .x()
    //     .onTrue(
    //         new frc.robot.commands.control.ShootNoteSequence(
    //             SHOOTER, WRIST, Constants.SHOOTER_RPM));
    DRIVER_CONTROLLER
        .leftBumper()
        .onTrue(
            new frc.robot.commands.control.IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR)
                .andThen(
                    new InstantCommand(
                            () -> {
                              DRIVER_CONTROLLER.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                              CANDLES.setColor(0, 155, 0);
                            })
                        .andThen(new WaitCommand(0.7))
                        .andThen(
                            new InstantCommand(
                                () -> {
                                  CANDLES.setColor(0, 155, 0);
                                  DRIVER_CONTROLLER.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                                }))));
    // driverController
    //     .leftBumper()
    //     .onTrue(new InstantCommand(() -> SHOOTER.setRPMShoot(Constants.SHOOTER_RPM)));

    DRIVER_CONTROLLER.rightBumper().onTrue(new ShootNote(SHOOTER, ELEVATOR, Constants.SHOOTER_RPM));
    DRIVER_CONTROLLER.a().onTrue(new ShootAmp(SHOOTER, ELEVATOR, WRIST));
    // .andThen(
    //     new InstantCommand(
    //         () ->s
    //             COMMANDS_TAB.addDouble(
    //                 "Distance of Last Shot",
    //                 () ->
    //                     LimelightHelpers.calculateDistanceToTarget(
    //                         LimelightHelpers.getTY(Constants.LIMELIGHT_SCORING),
    //                         Constants.SHOOTER_LIMELIGHT_HEIGHT,
    //                         1.45,
    //                         Constants.SHOOTER_LIMELIGHT_ANGLE)))));

    if (Utils.isSimulation()) {
      DRIVETRAIN.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    DRIVETRAIN.registerTelemetry(logger::telemeterize);
  }

  public void setupShuffleboard() {
    SHOOTER_TAB.add("Coast Wrist", new InstantCommand(() -> WRIST.coast()));
    SHOOTER_TAB.add("Brake Wrist", new InstantCommand(() -> WRIST.brake()));
    COMMANDS_TAB.add("Close Gates", new MoveGates(CLIMBER, true));
    COMMANDS_TAB.add("Open Gates", new MoveGates(CLIMBER, false));
    MATCH_TAB.addBoolean("Center LineBreak", () -> SHOOTER.isCenterBroken()).withPosition(0, 0);
    MATCH_TAB.addBoolean("Rear LineBreak", () -> SHOOTER.isRearBroken()).withPosition(0, 0);
    MATCH_TAB
        .add("Reverse Intake", new ReverseIntake(SHOOTER, INTAKE, WRIST, ELEVATOR))
        .withPosition(1, 0);
    MATCH_TAB
        .add("Increment Wrist +10 deg", new InstantCommand(() -> WRIST.incrementWrist(10)))
        .withPosition(1, 1);
    MATCH_TAB
        .add("Increment Wrist -10 deg", new InstantCommand(() -> WRIST.incrementWrist(-10)))
        .withPosition(2, 1);
    MATCH_TAB
        .addDouble("Increment Wrist Value", () -> WRIST.getIncrementValue())
        .withPosition(3, 1);
    MATCH_TAB
        .add("Increment Elevator +1 in", new InstantCommand(() -> ELEVATOR.incrementElevator(1)))
        .withPosition(1, 2);
    MATCH_TAB
        .add("Increment Elevator -1 in.", new InstantCommand(() -> ELEVATOR.incrementElevator(-1)))
        .withPosition(2, 2);
    MATCH_TAB
        .addDouble("Increment Elevator Value", () -> ELEVATOR.getIncrementValue())
        .withPosition(3, 2);
    MATCH_TAB.add(
        "InstaSuck",
        new InstantCommand(
                () -> {
                  SHOOTER.setFeederVoltage(Constants.FEEDER_FEEDFWD_VOLTS);
                  INTAKE.setVolts(Constants.INTAKE_COLLECT_VOLTS);
                })
            .andThen(new WaitCommand(0.1))
            .andThen(
                new InstantCommand(
                    () -> {
                      SHOOTER.stopFeeder();
                      INTAKE.stopTop();
                      INTAKE.stopCollecting();
                    })));

    COMMANDS_TAB.add("Shoot Amp", new ShootAmp(SHOOTER, ELEVATOR, WRIST));
    COMMANDS_TAB.add("Shoot Trap", new ShootTrap(ELEVATOR, WRIST, SHOOTER));
    COMMANDS_TAB.add(
        "Subwoofer Shot",
        new ShootNoteSequence(SHOOTER, WRIST, ELEVATOR, Constants.SHOOTER_RPM, 52, 2));
    POOP_RPM = COMMANDS_TAB.add("Poop RPM", 500).getEntry();
    COMMANDS_TAB.add("Poop Note", new PoopNote(SHOOTER, POOP_RPM.getDouble(500)));
    COMMANDS_TAB.addDouble(
        "Distance of Last Shot",
        () -> SHOOTER.ShooterCameraDistanceToTarget(Constants.SPEAKER_APRILTAG_HEIGHT));
    COMMANDS_TAB.add(
        "IntakeNote",
        new frc.robot.commands.control.IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR));
    COMMANDS_TAB.add("Set Wrist as at Home", new InstantCommand(() -> WRIST.zeroSensor()));
    COMMANDS_TAB.add(
        "ShootNote", new frc.robot.commands.control.ShootNoteSequence(SHOOTER, WRIST, 6000, 0));
    COMMANDS_TAB.add("SpinUpShooter", new InstantCommand(() -> SHOOTER.setRPMShoot(1800)));
    COMMANDS_TAB.add("StopShooter", new InstantCommand(() -> SHOOTER.setRPMShoot(0)));
    PHOTON_TAB.add("Rotate to AprilTag", new PointAtAprilTag(DRIVETRAIN, Constants.SPEAKER_PROTON));
    PHOTON_TAB.addDouble("Current Heading", () -> DRIVETRAIN.getPose().getRotation().getDegrees());
    PHOTON_TAB.addDouble("Current poseX", () -> DRIVETRAIN.getPose().getX());
    COMMANDS_TAB.add(
        "Driving Rotate to AprilTag",
        new PointAtAprilTag(
            DRIVETRAIN,
            Constants.SPEAKER_PROTON,
            () -> (DRIVER_CONTROLLER.getLeftY() * Constants.MaxSpeed),
            () -> (DRIVER_CONTROLLER.getLeftX() * Constants.MaxSpeed),
            () -> (DRIVER_CONTROLLER.getRightX() * Constants.MaxSpeed)));
    PHOTON_TAB.add(
        "Square Up AprilTag",
        new SquareUpToAprilTag(
            DRIVETRAIN, Constants.SPEAKER_PROTON, Constants.SPEAKER_APRILTAG_HEIGHT));
    PHOTON_TAB.add(
        "Climb Test",
        new SquareUpToAprilTag(DRIVETRAIN, Constants.SPEAKER_PROTON, Constants.TRAP_APRILTAG_HEIGHT)
        // .andThen(
        //     new InstantCommand(
        //             () ->
        //                 drivetrain.resetPose(
        //                     new Pose2d(2, 7, new Rotation2d(0)))) // Starting position of path
        //         .andThen(drivetrain.followPathCommand("Taxi")))
        );
    // LIMELIGHT_TAB.add(
    //     "Taxi path",
    //     new InstantCommand(
    //             () ->
    //                 DRIVETRAIN.seedFieldRelative(
    //                     new Pose2d(2, 7, new Rotation2d(0)))) // Starting position of path
    //         .andThen(DRIVETRAIN.("Taxi")));

    PHOTON_TAB.add(
        "Drive To Note",
        new DriveToNote(DRIVETRAIN, () -> -DRIVER_CONTROLLER.getLeftY(), Constants.INTAKE_PROTON));
    PHOTON_TAB.add("Drive To Note Auto", new DriveToNoteAuto(DRIVETRAIN, Constants.INTAKE_PROTON));
    PHOTON_TAB.add(
        "Collect Note Auto",
        new CollectNoteAuto(DRIVETRAIN, SHOOTER, INTAKE, WRIST, ELEVATOR, Constants.INTAKE_PROTON));

    SHOOTER_TAB.add("Spit Note", new SpitNote(SHOOTER));

    // LIMELIGHT_TAB.addNumber("Skew", () -> limelight.getLimelightNTDouble(limelight_scoring,
    // "ts"));

    // SmartDashboard.putData("Taxi", autoChooser);
    // SmartDashboard.putData("3 Piece 1", autoChooser);
    // SmartDashboard.putData("3PieceFar", autoChooser);
    autoChooser.addOption("3 Piece Far", new PathPlannerAuto("3 Piece Far"));
    autoChooser.addOption("temp", new PathPlannerAuto("temp"));
    autoChooser.addOption("ampa", new PathPlannerAuto("ampa"));
    autoChooser.addOption("sourcea", new PathPlannerAuto("sourcea"));
    autoChooser.addOption("amp_close", new PathPlannerAuto("amp_close"));
    COMMANDS_TAB.add(autoChooser);
  }

  public RobotContainer() {
    CANDLES.setColor(25, 0, 0);
    instance = this;
    registerNamedCommands();
    setupShuffleboard();
    configureBindings();

    WRIST.setDefaultCommand(new AimLockWrist(WRIST, SHOOTER, ELEVATOR));
    SHOOTER.setDefaultCommand(new IdleShooter(SHOOTER));
  }

  public static RobotContainer get() {
    return instance;
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand(
        "ShootNote", new ShootNoteSequence(SHOOTER, WRIST, Constants.SHOOTER_RPM, 40));
    NamedCommands.registerCommand("ShootNoteAimbot", new ShootNoteSequence(SHOOTER, WRIST, 5200));
    NamedCommands.registerCommand(
        "SpinUpShooter", new InstantCommand(() -> SHOOTER.setRPMShoot(5200)));
    NamedCommands.registerCommand(
        "ShootNoteSubFar",
        new ShootNoteSequence(SHOOTER, WRIST, ELEVATOR, Constants.SHOOTER_RPM, 36, 10));
    NamedCommands.registerCommand(
        "IntakeNote", new IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR));
    NamedCommands.registerCommand(
        "PoopStart",
        new InstantCommand(
            () -> {
              SHOOTER.setRPMShootNoSpin(850);
              SHOOTER.setFeederVoltage(7.0);
              INTAKE.setVolts(-8.0);
              WRIST.setDegrees(15);
            },
            SHOOTER,
            INTAKE));
    NamedCommands.registerCommand(
        "PoopStop",
        new InstantCommand(
            () -> {
              SHOOTER.stopShooter();
              SHOOTER.stopFeeder();
              INTAKE.stopCollecting();
            },
            SHOOTER,
            INTAKE));
    // NamedCommands.registerCommand("ResetOdo", new InstantCommand(() ->
    // DRIVETRAIN.seedFieldRelative()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // .andThen(
    //     new InstantCommand(() -> SHOOTER.setRPMShoot(Constants.SHOOTER_RPM), SHOOTER)
    //         .andThen(new WaitUntilCommand(() -> SHOOTER.isShooterAtSetpoint()))
    //         .andThen(new InstantCommand(() ->
    // SHOOTER.setFeederVoltage(Constants.FEEDER_SHOOT_VOLTS), SHOOTER))
    //         .andThen(new WaitCommand(0.25))
    //         .andThen(new InstantCommand(() -> {
    //             SHOOTER.stopFeeder();
    //             SHOOTER.stopShooter();
    //         }, SHOOTER))
    // );
    // return Commands.print("No autonomous command configured");
  }

  public static final double DEADBAND = 0.05;

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value input value
   * @return square of the value
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
