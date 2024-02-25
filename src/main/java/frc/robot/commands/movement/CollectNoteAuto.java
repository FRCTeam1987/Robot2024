// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.movement;

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
import frc.robot.commands.control.IntakeNoteSequence;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

public class CollectNoteAuto extends Command {
  /** Creates a new DriveToPiece. */
  private final Drivetrain drivetrain;

  private final Shooter shooter;
  private final Intake intake;
  private final Wrist wrist;
  private final Elevator elevator;
  private final Debouncer hasNote = new Debouncer(0.02, DebounceType.kRising);

  private static final String limelight = "limelight-intake";

  private Pose2d initialPose;
  private static final double kP = 0.07; // PID proportional gain
  private static final double kI = 0.00; // PID integral gain
  private static final double kD = 0.00; // PID derivative gain
  private static final double kToleranceDegrees = 0.1; // Tolerance for reaching the desired angle
  private static final double maximumAllowableDistance = 3.0; // In Meters
  private static final double slowDownDistance = 1.0; // Robot goes half speed once passed

  private final double ACCEPTABLE_DISTANCE = -0.2;
  private double distanceError;
  private final PIDController DISTANCE_CONTROLLER = new PIDController(0.60, 1, 0.1);
  private final LinearFilter DISTANCE_FILTER = LinearFilter.movingAverage(8);
  private double distanceToTarget;
  private double targetHeight = 0.03; // 1.23

  private final PIDController rotationController;
  private SwerveRequest.ApplyChassisSpeeds swerveRequest = new SwerveRequest.ApplyChassisSpeeds();

  private Debouncer canSeePieceDebouncer;
  private static final double DEBOUNCE_TIME = 0.06;

  // TODO find correct value and change name  public DriveToNoteAuto(final CommandSwerveDrivetrain
  // drivetrain) {

  public CollectNoteAuto(
      final Drivetrain drivetrain, final Shooter shooter, final Intake intake, final Wrist wrist, final Elevator elevator) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    this.wrist = wrist;
    this.elevator = elevator;

    // Create the PID controller
    rotationController = new PIDController(kP, kI, kD);
    rotationController.setTolerance(kToleranceDegrees);

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
    distanceToTarget =
        LimelightHelpers.calculateDistanceToTarget(
            LimelightHelpers.getTY(limelight),
            Constants.INTAKE_LIMELIGHT_HEIGHT,
            targetHeight,
            Constants.INTAKE_LIMELIGHT_ANGLE);
    new IntakeNoteSequence(shooter, intake, wrist, elevator); // Start intaking immediately
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!canSeePieceDebouncer.calculate(LimelightHelpers.getTV(limelight))) {
      DriverStation.reportWarning("DriveToPiece Can't see gamePicee", false);
      System.out.println("DriveToNote Can't see gamePice");
      drivetrain.setControl(swerveRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
      return;
    }

    double rotationalVelocity =
        rotationController.calculate(LimelightHelpers.getTX(limelight), 0.0);

    distanceToTarget =
        LimelightHelpers.calculateDistanceToTarget(
            LimelightHelpers.getTY(limelight),
            Constants.INTAKE_LIMELIGHT_HEIGHT,
            targetHeight,
            Constants.INTAKE_LIMELIGHT_ANGLE);

    distanceError = distanceToTarget - ACCEPTABLE_DISTANCE;

    double forwardBackwardSpeed =
        -DISTANCE_FILTER.calculate(DISTANCE_CONTROLLER.calculate(distanceError));

    double speed =
        distanceTraveled() > slowDownDistance
            ? forwardBackwardSpeed / 1.5
            : forwardBackwardSpeed; // TODO edited speeds so that robot goes a resonable speed when
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

    // Backups such that the command drives too far the motors will still stop
    shooter.stopFeeder();
    shooter.stopShooter();
    intake.stopCollecting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDistanceTraveledTooFar() || hasNote.calculate(shooter.isLineBreakBroken());
  }

  private double distanceTraveled() {
    return Math.abs(
        drivetrain.getPose().getTranslation().getDistance(initialPose.getTranslation()));
  }

  private boolean isDistanceTraveledTooFar() {
    return Math.abs(distanceTraveled()) > maximumAllowableDistance;
  }
}
