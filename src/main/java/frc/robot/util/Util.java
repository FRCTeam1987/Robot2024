/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class Util {
  public static final double DEADBAND = 0.05;

  public static double ctreVelocityToLinearVelocity(
      final double ctreVelocity, final double ticksPerRevolution, final double circumference) {
    return ctreVelocityToRps(ctreVelocity, ticksPerRevolution) * circumference;
  }

  public static double ctreVelocityToRps(
      final double ctreVelocity, final double ticksPerRevolution) {
    return ctreVelocity * 10.0 / ticksPerRevolution;
  }

  public static boolean isWithinTolerance(
      double currentValue, double targetValue, double tolerance) {
    return Math.abs(currentValue - targetValue) <= tolerance;
  }

  public static double ticksToDistance(
      final double ticks, final double ticksPerRevolution, final double circumference) {
    return ticksToDistance(ticks, ticksPerRevolution, circumference, 1.0);
  }

  public static double ticksToDistance(
      final double ticks,
      final double ticksPerRevolution,
      final double circumference,
      final double postEncoderGearing) {
    return ticks / (ticksPerRevolution * postEncoderGearing) * circumference;
  }

  public static int distanceToTicks(
      final double distance,
      final double ticksPerRevolution,
      final double circumference,
      final double postEncoderGearing) {
    return rotationsToTicks(
        distanceToRotations(distance, ticksPerRevolution, circumference, postEncoderGearing));
  }

  public static double distanceToRotations(
      final double distance,
      final double ticksPerRevolution,
      final double circumference,
      final double postEncoderGearing) {
    return (distance / circumference) / (ticksPerRevolution * postEncoderGearing);
  }

  public static int rotationsToTicks(final double rotations) {
    return (int) (rotations * 2048.0); // FALCON_ENCODER_RESOLUTION
  }

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

  public static boolean canSeeTarget(String limelight) {
    return Limelight.getBotPoseEstimate_wpiBlue(limelight).tagCount > 0;
  }

  public static double getInterpolatedDistance(String limelight) {
    var result = Limelight.getBotPoseEstimate_wpiBlue(limelight);
    if (result.tagCount > 0) {
      for (Limelight.RawFiducial target : result.rawFiducials) {
        if (target.id == 4) {
           return target.distToRobot;
        } else if (target.id == 7) {
           return target.distToRobot;
        }
      }
      return 0.0;
    }
    return 0.0;
  }

  public static double getInterpolatedWristAngle(String limelight) {
    return Constants.DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.getInterpolated(
            new InterpolatingDouble(Util.getInterpolatedDistance(limelight)))
        .value;
  }

  public static boolean isValidShot(String limelight) {
    double dist = Util.getInterpolatedDistance(limelight);
    if (dist > 2.25 && dist < 4.25) {
      return true;
    } else return false;
  }

  public static double squareValue(double value) {
    return Math.copySign(Math.pow(value, 2), value);
  }
}
