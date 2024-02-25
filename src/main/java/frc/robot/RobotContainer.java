// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.control.AimLockWrist;
import frc.robot.commands.control.IntakeNoteSequence;
import frc.robot.commands.control.LockWristAndPoint;
import frc.robot.commands.control.PoopNote;
import frc.robot.commands.control.ShootNote;
import frc.robot.commands.control.ShootNoteSequence;
import frc.robot.commands.control.SpitNote;
import frc.robot.commands.movement.CollectNoteAuto;
import frc.robot.commands.movement.DriveToNote;
import frc.robot.commands.movement.DriveToNoteAuto;
import frc.robot.commands.movement.PointAtAprilTag;
import frc.robot.commands.movement.SquareUpToAprilTag;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

public class RobotContainer {
  private static RobotContainer instance;
  private double MaxSpeed = 5.0; // 6 meters per second desired top speed
  private double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  public final ShuffleboardTab COMMANDS_TAB = Shuffleboard.getTab("COMMANDS");
  public static GenericEntry SHOOT_ANGLE;
  public static GenericEntry POOP_RPM;
  public final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("LIMELIGHT");
  public final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("SHOOTER");

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController =
      new CommandXboxController(0); // My joystick
  public final Drivetrain DRIVETRAIN = DriveConstants.DriveTrain; // My drivetrain

  public final Intake INTAKE = new Intake(Constants.INTAKE_TOP_ID, Constants.INTAKE_BOTTOM_ID);
  public final Shooter SHOOTER =
      new Shooter(
          Constants.SHOOTER_LEADER_ID, Constants.SHOOTER_FOLLOWER_ID, Constants.SHOOTER_FEEDER_ID);
  public final Wrist WRIST = new Wrist(Constants.WRIST_ID);
  public final Elevator ELEVATOR =
      new Elevator(Constants.ELEVATOR_LEADER_ID, Constants.ELEVATOR_FOLLOWER_ID);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final SendableChooser<Command> autoChooser;

  private final SlewRateLimiter translationXSlewRate = new SlewRateLimiter(4.0);
  private final SlewRateLimiter translationYSlewRate = new SlewRateLimiter(4.0);
  private final SlewRateLimiter rotationSlewRate = new SlewRateLimiter(4.0);

  private void configureBindings() {
    DRIVETRAIN.setDefaultCommand( // Drivetrain will execute this command periodically
        DRIVETRAIN.applyRequest(
            () ->
                drive
                    .withVelocityX(translationXSlewRate.calculate(-driverController.getLeftY()) * MaxSpeed) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(translationYSlewRate.calculate(
                        -driverController.getLeftX())
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(rotationSlewRate.calculate(
                        -driverController.getRightX())
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    driverController.a().whileTrue(DRIVETRAIN.applyRequest(() -> brake));
    driverController
        .b()
        .whileTrue(
            DRIVETRAIN.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.back().onTrue(DRIVETRAIN.runOnce(() -> DRIVETRAIN.seedFieldRelative()));

    driverController
        .rightTrigger()
        .whileTrue(
            new PointAtAprilTag(
                DRIVETRAIN,
                Constants.LIMELIGHT,
                Constants.LIMELIGHT_SCORING,
                () -> (driverController.getLeftY() * MaxSpeed),
                () -> (driverController.getLeftX() * MaxSpeed)));

    driverController
        .x()
        .onTrue(
            new frc.robot.commands.control.ShootNoteSequence(
                SHOOTER, WRIST, Constants.SHOOTER_RPM, 0));
    driverController
        .rightBumper()
        .onTrue(new frc.robot.commands.control.IntakeNoteSequence(SHOOTER, INTAKE, WRIST));
    driverController
        .leftBumper()
        .onTrue(new InstantCommand(() -> SHOOTER.setRPMShoot(Constants.SHOOTER_RPM)));

    driverController.y().onTrue(new ShootNote(SHOOTER, Constants.SHOOTER_RPM));
    // .andThen(
    //     new InstantCommand(
    //         () ->
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
    POOP_RPM = COMMANDS_TAB.add("Poop RPM", 1000).getEntry();
    COMMANDS_TAB.add("Poop Note", new PoopNote(SHOOTER, POOP_RPM.getDouble(1000)));
    COMMANDS_TAB.add("LockWrist&Point", new LockWristAndPoint(SHOOTER, WRIST, DRIVETRAIN));
    COMMANDS_TAB.addDouble(
        "Distance of Last Shot",
        () -> SHOOTER.ShooterCameraDistanceToTarget(Constants.SPEAKER_APRILTAG_HEIGHT));
    COMMANDS_TAB.add(
        "IntakeNote", new frc.robot.commands.control.IntakeNoteSequence(SHOOTER, INTAKE, WRIST));
    COMMANDS_TAB.add("Set Wrist as at Home", new InstantCommand(() -> WRIST.zeroSensor()));
    SHOOT_ANGLE = COMMANDS_TAB.add("Shoot Angle", 30).getEntry();
    COMMANDS_TAB.add(
        "ShootNote", new frc.robot.commands.control.ShootNoteSequence(SHOOTER, WRIST, 6000, 0));
    COMMANDS_TAB.add("SpinUpShooter", new InstantCommand(() -> SHOOTER.setRPMShoot(1800)));
    COMMANDS_TAB.add("StopShooter", new InstantCommand(() -> SHOOTER.setRPMShoot(0)));
    LIMELIGHT_TAB.add(
        "Rotate to AprilTag",
        new PointAtAprilTag(DRIVETRAIN, Constants.LIMELIGHT, Constants.LIMELIGHT_SCORING));
    LIMELIGHT_TAB.addDouble(
        "Current Heading", () -> DRIVETRAIN.getPose().getRotation().getDegrees());
    LIMELIGHT_TAB.addDouble("Current poseX", () -> DRIVETRAIN.getPose().getX());
    COMMANDS_TAB.add(
        "Driving Rotate to AprilTag",
        new PointAtAprilTag(
            DRIVETRAIN,
            Constants.LIMELIGHT,
            Constants.LIMELIGHT_SCORING,
            () -> (-driverController.getLeftX() * MaxSpeed),
            () -> (-driverController.getLeftY() * MaxSpeed)));
    LIMELIGHT_TAB.add(
        "Square Up AprilTag",
        new SquareUpToAprilTag(
            DRIVETRAIN, Constants.LIMELIGHT_SCORING, Constants.SPEAKER_APRILTAG_HEIGHT));
    LIMELIGHT_TAB.add(
        "Climb Test",
        new SquareUpToAprilTag(
            DRIVETRAIN, Constants.LIMELIGHT_SCORING, Constants.TRAP_APRILTAG_HEIGHT)
        // .andThen(
        //     new InstantCommand(
        //             () ->
        //                 drivetrain.resetPose(
        //                     new Pose2d(2, 7, new Rotation2d(0)))) // Starting position of path
        //         .andThen(drivetrain.followPathCommand("Taxi")))
        );
    LIMELIGHT_TAB.add(
        "Taxi path",
        new InstantCommand(
                () ->
                    DRIVETRAIN.resetPose(
                        new Pose2d(2, 7, new Rotation2d(0)))) // Starting position of path
            .andThen(DRIVETRAIN.followPathCommand("Taxi")));

    LIMELIGHT_TAB.add(
        "Drive To Note", new DriveToNote(DRIVETRAIN, () -> -driverController.getLeftY()));
    LIMELIGHT_TAB.add("Drive To Note Auto", new DriveToNoteAuto(DRIVETRAIN));
    LIMELIGHT_TAB.add("Collect Note Auto", new CollectNoteAuto(DRIVETRAIN, SHOOTER, INTAKE, WRIST));

    SHOOTER_TAB.add("Spit Note", new SpitNote(SHOOTER));

    // LIMELIGHT_TAB.addNumber("Skew", () -> limelight.getLimelightNTDouble(limelight_scoring,
    // "ts"));

    SmartDashboard.putData("Taxi", autoChooser);
    SmartDashboard.putData("3 Piece 1", autoChooser);
  }

  public RobotContainer() {
    instance = this;
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    setupShuffleboard();
    // WRIST.setDefaultCommand(new AimLockWrist(WRIST));
  }

  public static RobotContainer get() {
    return instance;
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand(
        "ShootNote", new ShootNoteSequence(SHOOTER, WRIST, Constants.SHOOTER_RPM, 40));
    NamedCommands.registerCommand("IntakeNote", new IntakeNoteSequence(SHOOTER, INTAKE, WRIST));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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
