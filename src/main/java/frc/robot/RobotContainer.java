// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.PointAtAprilTag;
import frc.robot.commands.SquareUpToAprilTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private LimelightHelpers limelight = new LimelightHelpers();
  public String limelight_scoring = "limelight-scoring";
  public final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("SHOOTER");
  public final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("LIMELIGHT");
  // private final ShuffleboardTab INTAKE_TAB = Shuffleboard.getTab("INTAKE");

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    joystick.rightTrigger().onTrue(new SquareUpToAprilTag(drivetrain, limelight_scoring));

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
            () -> (-joystick.getLeftX() * MaxSpeed),
            () -> (-joystick.getLeftY() * MaxSpeed)));
    LIMELIGHT_TAB.add("Square Up AprilTag", new SquareUpToAprilTag(drivetrain, limelight_scoring));
    LIMELIGHT_TAB.addNumber("Skew", () -> limelight.getLimelightNTDouble(limelight_scoring, "ts"));
  }

  public RobotContainer() {
    configureBindings();
    setupShuffleboard();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
