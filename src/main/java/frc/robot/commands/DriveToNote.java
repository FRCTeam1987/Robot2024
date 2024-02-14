// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToNote extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private Pose2d initialPose;
  private static final double maximumAllowableDistance = 3.0; // In Meters

  private SwerveRequest.ApplyChassisSpeeds swerveRequest = new SwerveRequest.ApplyChassisSpeeds();
  private final LinearFilter LATERAL_FILTER = LinearFilter.movingAverage(15);
  private final LinearFilter DISTANCE_FILTER = LinearFilter.movingAverage(8);

  private final PIDController LATERAL_CONTROLLER = new PIDController(0.1, 0.001, 0.01);
  private final PIDController ROTATIONAL_CONTROLLER = new PIDController(0.15, 0, 0.01);
  private final PIDController DISTANCE_CONTROLLER = new PIDController(0.60, 1, 0.1);

  private final String limelight = "limelight-intake";
  private double cameraHeight = 0.13;
  private double targetHeight = 0; // looking for notes on the floor
  private double cameraAngle = 35;
  private final double ACCEPTABLE_DISTANCE = 3; // 1.3

  private Debouncer canSeePieceDebouncer;
  private static final double DEBOUNCE_TIME = 0.06; // TODO find correct value and change name

  /** Creates a new DriveToNote. */
  public DriveToNote(final CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;

    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.initialPose = drivetrain.getPose();
    // drivetrain.disableFieldRelative();
    // drivetrain.disableXstance();

    canSeePieceDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!canSeePieceDebouncer.calculate(LimelightHelpers.getTV(limelight))) {
      // DriverStation.reportWarning("DriveToNote Can't see gamePicee", false);
      System.out.println("DriveToNote Can't see gamePice");
      drivetrain.setControl(swerveRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
      return;
    }

    double distanceToTarget =
        LimelightHelpers.calculateDistanceToTarget(
            LimelightHelpers.getTY(limelight), cameraHeight, targetHeight, cameraAngle);
    System.out.println("distanceToTarget = " + distanceToTarget);
    double distanceError = distanceToTarget - ACCEPTABLE_DISTANCE;
    double forwardBackwardSpeed = DISTANCE_CONTROLLER.calculate(distanceError);
    double xOffset = LimelightHelpers.getTX(limelight);
    double skew = LimelightHelpers.getLimelightNTDouble(limelight, "ts");

    double rotationRate = ROTATIONAL_CONTROLLER.calculate(-xOffset);
    double lateralSpeed = LATERAL_CONTROLLER.calculate(skew);

    swerveRequest =
        swerveRequest.withSpeeds(
            new ChassisSpeeds(
                -DISTANCE_FILTER.calculate(forwardBackwardSpeed),
                LATERAL_FILTER.calculate(lateralSpeed),
                -rotationRate));

    // Apply the request to the drivetrain
    drivetrain.setControl(swerveRequest);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO If it fails to pickup gamepiece send arm home
    DriverStation.reportWarning("DriveToNote Command Finished", false);
    System.out.println("DriveToPiece Command Finished");
    // drivetrain.enableFieldRelative();
    drivetrain.setControl(swerveRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    if (isDistanceTraveledTooFar()) {
      DriverStation.reportWarning("DriveToNote Drove Too Far", false);
      System.out.println("DriveToPiece Drove Too Far");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDistanceTraveledTooFar();
  }

  private double distanceTraveled() {
    return Math.abs(
        drivetrain.getPose().getTranslation().getDistance(initialPose.getTranslation()));
  }

  private boolean isDistanceTraveledTooFar() {
    return Math.abs(distanceTraveled()) > maximumAllowableDistance;
  }
}
