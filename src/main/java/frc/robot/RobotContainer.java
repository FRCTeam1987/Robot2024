// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveToNote;
import frc.robot.commands.PointAtAprilTag;
import frc.robot.commands.SquareUpToAprilTag;
import frc.robot.generated.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private LimelightHelpers limelight = new LimelightHelpers();
  public String limelight_scoring = "limelight-scoring";
  public final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("SHOOTER");
  public final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("LIMELIGHT");
  private final ShuffleboardTab INTAKE_TAB = Shuffleboard.getTab("INTAKE");

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController =
      new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Intake INTAKE = new Intake(Constants.INTAKE_OUT_ID, Constants.INTAKE_IN_ID);

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

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(
                        -driverController.getLeftX()
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -driverController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    driverController.rightTrigger().onTrue(new SquareUpToAprilTag(drivetrain, limelight_scoring));

    driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void setupShuffleboard() {
    LIMELIGHT_TAB.add(
        "Rotate to AprilTag", new PointAtAprilTag(drivetrain, limelight, limelight_scoring));
    LIMELIGHT_TAB.add(
        "Driving Rotate to AprilTag",
        new PointAtAprilTag(
            drivetrain,
            limelight,
            limelight_scoring,
            () -> (-driverController.getLeftX() * MaxSpeed),
            () -> (-driverController.getLeftY() * MaxSpeed)));
    LIMELIGHT_TAB.add("Square Up AprilTag", new SquareUpToAprilTag(drivetrain, limelight_scoring));
    LIMELIGHT_TAB.add(
        "Climb Test", new SquareUpToAprilTag(drivetrain, limelight_scoring)
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
                    drivetrain.resetPose(
                        new Pose2d(2, 7, new Rotation2d(0)))) // Starting position of path
            .andThen(drivetrain.followPathCommand("Taxi")));
    
    LIMELIGHT_TAB.add("Drive To Note", new DriveToNote(drivetrain));

    // LIMELIGHT_TAB.addNumber("Skew", () -> limelight.getLimelightNTDouble(limelight_scoring,
    // "ts"));
    // LIMELIGHT_TAB.addNumber("Distance", () ->
    // LimelightHelpers.calculateDistanceToTarget(LimelightHelpers.getTY(limelight_scoring), 0.13,
    // 1.23, 35));

    SmartDashboard.putData("Taxi", autoChooser);
  }

  public RobotContainer() {
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    setupShuffleboard();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return Commands.print("No autonomous command configured");
  }
}
