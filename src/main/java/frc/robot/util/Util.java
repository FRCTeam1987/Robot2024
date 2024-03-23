/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

public class Util {
  public static final AprilTagFieldLayout field =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  public static Pose3d TAG_4_POSE;
  public static final double DEADBAND = 0.05;

  public Util() {
    field
        .getTagPose(4)
        .ifPresent(
            pose -> {
              Util.TAG_4_POSE = pose;
            });
  }

  public static boolean isWithinTolerance(
      double currentValue, double targetValue, double tolerance) {
    return Math.abs(currentValue - targetValue) <= tolerance;
  }

  public static boolean canSeeTarget(String limelight) {
    return Limelight.getBotPoseEstimate_wpiBlue(limelight).tagCount > 0;
  }

  public static void setupUtil() {}

  public static double getDistance(String limelight) {
    var result = Limelight.getBotPoseEstimate_wpiBlue(limelight);
    if (result.tagCount > 0) {
      return new Pose3d(RobotContainer.get().getPose())
          .getTranslation()
          .getDistance(
              TAG_4_POSE
                  .transformBy(
                      new Transform3d(
                          new Translation3d(0, 0, -TAG_4_POSE.getZ()), new Rotation3d()))
                  .getTranslation());
    }
    return 0.0;
  }

  public static double getInterpolatedWristAngle(String limelight) {
    return Constants.DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.getInterpolated(
            new InterpolatingDouble(Util.getDistance(limelight)))
        .value;
  }

  public static boolean isValidShot(String limelight) {
    double dist = Util.getDistance(limelight);
    if (dist > 2.25 && dist < 4.25) {
      return true;
    } else return false;
  }

  public static double squareValue(double value) {
    return Math.copySign(Math.pow(value, 2), value);
  }
}
