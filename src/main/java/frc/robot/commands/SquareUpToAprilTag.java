// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SquareUpToAprilTag extends Command {
  private String LOG_PREFIX = "[EXECUTE] ";

  private final PIDController LATERAL_CONTROLLER = new PIDController(0.3, 0.001, 0.01);
  private final PIDController ROTATIONAL_CONTROLLER = new PIDController(0.22, 0, 0.01);
  private final PIDController DISTANCE_CONTROLLER = new PIDController(1, 1, 0.1);

  private final LinearFilter LATERAL_FILTER = LinearFilter.movingAverage(15);
  private final LinearFilter DISTANCE_FILTER = LinearFilter.movingAverage(8);
  private final LinearFilter SKEW_FILTER = LinearFilter.movingAverage(8);

  private final double ACCEPTABLE_SKEW_ERROR = 0.3; // Degrees within acceptance
  private final double ACCEPTABLE_DISTANCE_ERROR = 2.5; // metersP within acceptance

  private String limeLightName;
  private double distanceError;
  private CommandSwerveDrivetrain drivetrain;
  private double xOffset;
  private double skew;
  private double distanceToTarget;
  private SwerveRequest.ApplyChassisSpeeds swerveRequest = new SwerveRequest.ApplyChassisSpeeds();
  private int noVisibleTargetLoops = 0;
  private double cameraHeight = 0.13;
  private double targetHeight = 1.23;
  private double cameraAngle = 35;

  public SquareUpToAprilTag(CommandSwerveDrivetrain drivetrain, String limeLightName) {
    this.drivetrain = drivetrain;
    this.limeLightName = limeLightName;
  }

  @Override
  public void initialize() {
    System.out.println("Squaring up to AprilTag...");
    xOffset = LimelightHelpers.getTX(limeLightName);
    skew = LimelightHelpers.getLimelightNTDouble(limeLightName, "ts");
    distanceToTarget =
        LimelightHelpers.calculateDistanceToTarget(
            LimelightHelpers.getTY(limeLightName), cameraHeight, targetHeight, cameraAngle);
  }

  // @Override
  // public void execute() { //One direction at a time, slower but works and is consistent
  //   System.out.println(LOG_PREFIX + "In execute()");
  //   if (!LimelightHelpers.getTV(limeLightName)) {
  //     System.out.println(LOG_PREFIX + "No visible target.");
  //     noVisibleTargetLoops++;
  //   } else {
  //     noVisibleTargetLoops = 0;
  //     xOffset = LimelightHelpers.getTX(limeLightName);
  //     skew = LimelightHelpers.getLimelightNTDouble(limeLightName, "ts");
  //     distanceToTarget =
  //         LimelightHelpers.calculateDistanceToTarget(
  //             LimelightHelpers.getTY(limeLightName), cameraHeight, targetHeight, cameraAngle);

  //     if (skew > 70) {
  //       skew = skew - 90;
  //     }

  //     skew = SKEW_FILTER.calculate(skew);

  //     System.out.println(LOG_PREFIX + "Skew: " + skew);

  //     double forwardBackwardSpeed = 0;
  //     double rotationRate = 0;
  //     double lateralSpeed = 0;

  //     if (Math.abs(skew) < ACCEPTABLE_SKEW_ERROR) {
  //       System.out.println(LOG_PREFIX + "Acceptable skew. Driving forward..");
  //       distanceError = distanceToTarget - ACCEPTABLE_DISTANCE_ERROR;
  //       forwardBackwardSpeed = DISTANCE_CONTROLLER.calculate(distanceError);
  //     } else {
  //       System.out.println(LOG_PREFIX + "Correcting skew & lateral position");
  //       rotationRate = ROTATIONAL_CONTROLLER.calculate(-xOffset);
  //       lateralSpeed = LATERAL_CONTROLLER.calculate(skew);
  //     }

  //     swerveRequest =
  //         swerveRequest.withSpeeds(
  //             new ChassisSpeeds(
  //                 -DISTANCE_FILTER.calculate(forwardBackwardSpeed),
  //                 LATERAL_FILTER.calculate(lateralSpeed),
  //                 -rotationRate));

  //     // Apply the request to the drivetrain
  //     drivetrain.setControl(swerveRequest);
  //   }
  // }

  @Override
  public void execute() { //Run both directions at once, needs tested but should be faster
      System.out.println(LOG_PREFIX + "In execute()");
      if (!LimelightHelpers.getTV(limeLightName)) {
          System.out.println(LOG_PREFIX + "No visible target.");
          noVisibleTargetLoops++;
      } else {
          noVisibleTargetLoops = 0;
          xOffset = LimelightHelpers.getTX(limeLightName);
          skew = LimelightHelpers.getLimelightNTDouble(limeLightName, "ts");
          distanceToTarget = LimelightHelpers.calculateDistanceToTarget(
                  LimelightHelpers.getTY(limeLightName), cameraHeight, targetHeight, cameraAngle);
  
          if (skew > 70) {
              skew = skew - 90;
          }
  
          skew = SKEW_FILTER.calculate(skew);
  
          System.out.println(LOG_PREFIX + "Skew: " + skew);
  
          double forwardBackwardSpeed = 0;
          double rotationRate = 0;
          double lateralSpeed = 0;
  
          if (Math.abs(skew) < ACCEPTABLE_SKEW_ERROR) {
              System.out.println(LOG_PREFIX + "Acceptable skew. Driving forward..");
              distanceError = distanceToTarget - ACCEPTABLE_DISTANCE_ERROR;
              forwardBackwardSpeed = DISTANCE_CONTROLLER.calculate(distanceError);
          }
  
          // Always correct lateral position
          rotationRate = ROTATIONAL_CONTROLLER.calculate(-xOffset);
          lateralSpeed = LATERAL_CONTROLLER.calculate(skew);
  
          swerveRequest = swerveRequest.withSpeeds(
                  new ChassisSpeeds(
                          -DISTANCE_FILTER.calculate(forwardBackwardSpeed),
                          LATERAL_FILTER.calculate(lateralSpeed),
                          -rotationRate));
  
          // Apply the request to the drivetrain
          drivetrain.setControl(swerveRequest);
      }
  }

  // Code Suggestion: Make the robot return to last position it saw the tag if it gets lost

  @Override
  public boolean isFinished() {
    if (((distanceToTarget < ACCEPTABLE_DISTANCE_ERROR) && Math.abs(skew) <= ACCEPTABLE_SKEW_ERROR)
        || (noVisibleTargetLoops == 10)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println(LOG_PREFIX + "Reached end()");
    drivetrain.setControl(swerveRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));

    if (interrupted) {}
  }
}
