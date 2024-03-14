// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.movement;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class DriveToNote extends Command {
  private static final double P = 0.07; // PID proportional gain
  private static final double I = 0.00; // PID integral gain
  private static final double D = 0.00; // PID derivative gain
  private static final double TOLERANCE_DEGREES = 0.1; // Tolerance for reaching the desired angle
  private static final double maximumAllowableDistance = 3.0; // In Meters
  private static final double slowDownDistance = 1.0; // Robot goes half speed once passed
  private static final double DEBOUNCE_TIME = 0.06; // TODO find correct value and change name

  /** Creates a new DriveToPiece. */
  private final DoubleSupplier velocitySupplier;

  private final CommandSwerveDrivetrain drivetrain;
  private final Vision photonVision;
  private final PIDController rotationController;
  private final SwerveRequest.ApplyChassisSpeeds swerveRequest =
      new SwerveRequest.ApplyChassisSpeeds();
  private Pose2d initialPose;
  private Debouncer canSeePieceDebouncer;

  public DriveToNote(
      final CommandSwerveDrivetrain drivetrain, final DoubleSupplier velocitySupplier, Vision photonVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.velocitySupplier = velocitySupplier;
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;

    // Create the PID controller
    rotationController = new PIDController(P, I, D);
    rotationController.setTolerance(TOLERANCE_DEGREES);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveToNote - init");
    // Apply the output to the swerve
    this.initialPose = drivetrain.getPose();
    rotationController.reset();

    canSeePieceDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!canSeePieceDebouncer.calculate(photonVision.hasTargets())) {
      DriverStation.reportWarning("DriveToPiece Can't see gamePicee", false);
      System.out.println("DriveToNote Can't see gamePice");
      drivetrain.setControl(swerveRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
      return;
    }

    double rotationalVelocity = rotationController.calculate(photonVision.getYawVal(), 0.0);
    double speed =
        distanceTraveled() > slowDownDistance
            ? velocitySupplier.getAsDouble() / 2.0
            : velocitySupplier
                .getAsDouble(); // TODO edited speeds so that robot goes a resonable speed when
    // closer
    // System.out.println("========================= DriveToPiece Speed: " + speed);

    drivetrain.setControl(
        swerveRequest.withSpeeds(
            new ChassisSpeeds(
                speed, 0, rotationalVelocity))); // y position should not be edited  as it is driver
    // controlled.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("DriveToNote Command Finished", false);
    System.out.println("DriveToNote Command Finished");
    drivetrain.setControl(swerveRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    if (isDistanceTraveledTooFar()) {
      DriverStation.reportWarning("DriveToNote Drove Too Far", false);
      System.out.println("DriveToNote Drove Too Far");
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
