// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.movement;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.util.Util;

public class SquareUpToAprilTag extends Command {
  private final String LOG_PREFIX = "[EXECUTE] ";

  private final PIDController LATERAL_CONTROLLER = new PIDController(0.1, 0.001, 0.01);
  private final PIDController ROTATIONAL_CONTROLLER = new PIDController(0.15, 0, 0.01);
  private final PIDController DISTANCE_CONTROLLER = new PIDController(0.60, 1, 0.1);

  private final LinearFilter LATERAL_FILTER = LinearFilter.movingAverage(15);
  private final LinearFilter DISTANCE_FILTER = LinearFilter.movingAverage(8);
  private final LinearFilter SKEW_FILTER = LinearFilter.movingAverage(8);

  private final double ACCEPTABLE_SKEW_ERROR = 0.0; // Degrees within acceptance
  private final double ACCEPTABLE_DISTANCE_ERROR = 0.05; // metersP within acceptance
  private final Vision photonVision;
  private final CommandSwerveDrivetrain drivetrain;
  private double ACCEPTABLE_DISTANCE = 3; // 1.3
  private double distanceError;
  private double xOffset;
  private double skew;
  private double distanceToTarget;
  private SwerveRequest.ApplyChassisSpeeds swerveRequest = new SwerveRequest.ApplyChassisSpeeds();
  private int noVisibleTargetLoops = 0;
  private double targetHeight = 1.45; // 1.23 Meters

  public SquareUpToAprilTag(
      CommandSwerveDrivetrain drivetrain,
      Vision photonVision,
      double targetHeight,
      double acceptableDistance) {
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.targetHeight = targetHeight;
    this.ACCEPTABLE_DISTANCE = acceptableDistance;
  }

  @Override
  public void initialize() {
    System.out.println("Squaring up to AprilTag...");
    xOffset = photonVision.getYawVal();
    skew = photonVision.getSkewVal();
    distanceToTarget =
        Vision.calculateDistanceToTarget(
            photonVision.getPitchVal(),
            photonVision.getCameraHeight(),
            targetHeight,
            photonVision.getCameraDegrees());
  }

  @Override
  public void execute() {
    System.out.println(LOG_PREFIX + "In execute()");
    if (!photonVision.hasTargets()) {
      System.out.println(LOG_PREFIX + "No visible target.");
      noVisibleTargetLoops++;
      drivetrain.setControl(swerveRequest);

    } else {
      noVisibleTargetLoops = 0;
      xOffset = photonVision.getYawVal();
      skew = photonVision.getSkewVal();
      distanceToTarget =
          Vision.calculateDistanceToTarget(
              photonVision.getPitchVal(),
              photonVision.getCameraHeight(),
              targetHeight,
              photonVision.getCameraDegrees());

      if (skew > 70) {
        skew = skew - 90;
      }

      skew = SKEW_FILTER.calculate(skew);

      System.out.println(LOG_PREFIX + "Skew: " + skew);

      double forwardBackwardSpeed = 0;
      double rotationRate = 0;
      double lateralSpeed = 0;

      distanceError = distanceToTarget - ACCEPTABLE_DISTANCE;
      forwardBackwardSpeed = DISTANCE_CONTROLLER.calculate(distanceError);

      rotationRate = ROTATIONAL_CONTROLLER.calculate(-xOffset);
      lateralSpeed = LATERAL_CONTROLLER.calculate(skew);

      swerveRequest =
          swerveRequest.withSpeeds(
              new ChassisSpeeds(
                  -DISTANCE_FILTER.calculate(forwardBackwardSpeed),
                  LATERAL_FILTER.calculate(lateralSpeed),
                  -rotationRate));

      // Apply the request to the drivetrain
      drivetrain.setControl(swerveRequest);
    }
  }

  @Override
  public boolean isFinished() {
    return (Util.isWithinTolerance(distanceToTarget, ACCEPTABLE_DISTANCE, ACCEPTABLE_DISTANCE_ERROR)
            && Math.abs(skew) <= ACCEPTABLE_SKEW_ERROR)
        || (noVisibleTargetLoops >= 10);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println(LOG_PREFIX + "Reached end()");
    drivetrain.setControl(swerveRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));

    if (interrupted) {}
  }
}
