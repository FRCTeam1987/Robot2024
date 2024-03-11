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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.control.IntakeNoteSequence;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

public class DriveToNoteAuto extends Command {
  /** Creates a new DriveToPiece. */
  private final Drivetrain drivetrain;

  private final Intake intake;
  private final Shooter shooter;
  private static Vision photonVision;
  private static Wrist wrist;
  private static Elevator elevator;

  private Pose2d initialPose;
  private static final double kP = 0.07; // PID proportional gain
  private static final double kI = 0.00; // PID integral gain
  private static final double kD = 0.00; // PID derivative gain
  private static final double kToleranceDegrees = 0.1; // Tolerance for reaching the desired angle
  private static final double maximumAllowableDistance = 2.5; // In Meters
  private static final double slowDownDistance = 1.0; // Robot goes half speed once passed

  private final PIDController DISTANCE_CONTROLLER = new PIDController(0.60, 1, 0.1);
  private final LinearFilter DISTANCE_FILTER = LinearFilter.movingAverage(8);
  private double distanceToTarget;
  private final double targetHeight = 0.03; // 1.23

  private double previousForwardBackwardSpeed = 0.0;

  private final PIDController rotationController;
  private final SwerveRequest.ApplyChassisSpeeds swerveRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  private Debouncer canSeePieceDebouncer;
  private static final double DEBOUNCE_TIME = 0.3;

  // TODO find correct value and change name  public DriveToNoteAuto(final CommandSwerveDrivetrain
  // drivetrain) {

  public DriveToNoteAuto(
      final Drivetrain drivetrain,
      final Vision photonVision,
      final Shooter shooter,
      final Intake intake,
      final Wrist wrist,
      final Elevator elevator) {
    this.drivetrain = drivetrain;
    DriveToNoteAuto.photonVision = photonVision;
    this.shooter = shooter;
    this.intake = intake;
    DriveToNoteAuto.wrist = wrist;
    DriveToNoteAuto.elevator = elevator;

    // Create the PID controller
    rotationController = new PIDController(kP, kI, kD);
    rotationController.setTolerance(kToleranceDegrees);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.reportWarning("DriveToNote - init", false);

    // Apply the output to the swerve
    this.initialPose = drivetrain.getPose();
    rotationController.reset();

    canSeePieceDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);
    new IntakeNoteSequence(shooter, intake, wrist, elevator);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!canSeePieceDebouncer.calculate(photonVision.hasTargets())) {
      DriverStation.reportWarning("DriveToPiece Can't see gamePicee", false);
      // System.out.println("DriveToNote Can't see gamePice");
      drivetrain.setControl(
          swerveRequest.withSpeeds(new ChassisSpeeds(previousForwardBackwardSpeed, 0, 0)));
      return;
    }

    double rotationalVelocity = rotationController.calculate(photonVision.getYawVal(), 0.0);

    double ACCEPTABLE_DISTANCE = -0.2;
    double distanceError = distanceToTarget - ACCEPTABLE_DISTANCE;

    double forwardBackwardSpeed = 3.0; // meters per second

    previousForwardBackwardSpeed = forwardBackwardSpeed;

    // temp
    drivetrain.setControl(
        swerveRequest.withSpeeds(
            new ChassisSpeeds(
                forwardBackwardSpeed,
                0,
                rotationalVelocity))); // y position should not be edited  as it is driver
    // controlled.

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("DriveToNote Command Finished", false);
    drivetrain.setControl(swerveRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    new InstantCommand(
        () -> {
          shooter.stopFeeder();
          intake.stopCollecting();
          intake.stopTop();
        },
        shooter,
        intake);
    if (isDistanceTraveledTooFar()) {
      DriverStation.reportWarning("DriveToNote Drove Too Far", false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDistanceTraveledTooFar() || shooter.isRearBroken();
  }

  private double distanceTraveled() {
    return Math.abs(
        drivetrain.getPose().getTranslation().getDistance(initialPose.getTranslation()));
  }

  private boolean isDistanceTraveledTooFar() {
    return Math.abs(distanceTraveled()) > maximumAllowableDistance;
  }
}
