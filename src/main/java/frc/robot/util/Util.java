/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.util.Limelight.RawFiducial;
import java.util.List;

public class Util {
  public static final AprilTagFieldLayout field =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  public static Pose3d TAG_4_POSE; // RED ALLIANCE SPEAKER CENTER
  public static Pose3d TAG_7_POSE; // BLUE ALLIANCE SPEAKER CENTER
  public static Pose2d TAG_11_POSE_2D; // LEFT SIDE RED ALLIANCE TRAP
  public static Pose2d TAG_12_POSE_2D; // RIGHT SIDE RED ALLIANCE TRAP
  public static Pose2d TAG_13_POSE_2D; // FAR SIDE RED ALLIANCE TRAP
  public static Pose2d TAG_14_POSE_2D; // FAR SIDE BLUE ALLIANCE TRAP
  public static Pose2d TAG_15_POSE_2D; // LEFT SIDE BLUE ALLIANCE TRAP
  public static Pose2d TAG_16_POSE_2D; // RIGHT SIDE BLUE ALLIANCE TRAP
  public static List<Pose2d> TRAP_TAGS;
  public static DriverStation.Alliance alliance;
  public static final double DEADBAND = 0.05;

  public Util() {
    field
        .getTagPose(4)
        .ifPresent(
            pose -> {
              Util.TAG_4_POSE =
                  pose; // .transformBy(new Transform3d(0, 0.565, 0, new Rotation3d()));
            });
    field
        .getTagPose(7)
        .ifPresent(
            pose -> {
              Util.TAG_7_POSE =
                  pose; // .transformBy(new Transform3d(0, 0.565, 0, new Rotation3d()));
            });
    field
        .getTagPose(12)
        .ifPresent(
            pose -> {
              Util.TAG_12_POSE_2D =
                  pose.toPose2d(); // .transformBy(new Transform3d(0, 0.565, 0, new Rotation3d()));
            });
    field
        .getTagPose(11)
        .ifPresent(
            pose -> {
              Util.TAG_11_POSE_2D =
                  pose.toPose2d(); // .transformBy(new Transform3d(0, 0.565, 0, new Rotation3d()));
            });
    field
        .getTagPose(13)
        .ifPresent(
            pose -> {
              Util.TAG_13_POSE_2D =
                  pose.toPose2d(); // .transformBy(new Transform3d(0, 0.565, 0, new Rotation3d()));
            });
    field
        .getTagPose(14)
        .ifPresent(
            pose -> {
              Util.TAG_14_POSE_2D =
                  pose.toPose2d(); // .transformBy(new Transform3d(0, 0.565, 0, new Rotation3d()));
            });
    field
        .getTagPose(15)
        .ifPresent(
            pose -> {
              Util.TAG_15_POSE_2D =
                  pose.toPose2d(); // .transformBy(new Transform3d(0, 0.565, 0, new Rotation3d()));
            });
    field
        .getTagPose(16)
        .ifPresent(
            pose -> {
              Util.TAG_16_POSE_2D =
                  pose.toPose2d(); // .transformBy(new Transform3d(0, 0.565, 0, new Rotation3d()));
            });
    TRAP_TAGS =
        List.of(
            TAG_11_POSE_2D,
            TAG_12_POSE_2D,
            TAG_13_POSE_2D,
            TAG_14_POSE_2D,
            TAG_15_POSE_2D,
            TAG_16_POSE_2D);
    DriverStation.getAlliance().ifPresent(ouralliance -> alliance = ouralliance);
  }

  public static Pose3d getAllianceSpeakerCenter() {
    return alliance == DriverStation.Alliance.Blue ? TAG_7_POSE : TAG_4_POSE;
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
      return new Pose3d(result.pose)
          .getTranslation()
          .getDistance(
              getAllianceSpeakerCenter()
                  .transformBy(
                      new Transform3d(
                          new Translation3d(0, 0, -getAllianceSpeakerCenter().getZ()),
                          new Rotation3d()))
                  .getTranslation());
    }
    return 0.0;
  }

  // TODO flex on alliance tag pose
  public static double getDistanceToSpeaker() {
    return new Pose3d(RobotContainer.get().getPose())
        .getTranslation()
        .getDistance(
            getAllianceSpeakerCenter()
                .transformBy(
                    new Transform3d(
                        new Translation3d(0, 0, -getAllianceSpeakerCenter().getZ()),
                        new Rotation3d()))
                .getTranslation());
  }

  public static Rotation2d getRotationToAllianceSpeaker(Pose2d opose) {
    // return
    // opose.getTranslation().minus(Util.getAllianceSpeakerCenter().getTranslation().toTranslation2d()).getAngle();
    Transform2d pose = Util.getAllianceSpeakerCenter().toPose2d().minus(opose);
    return new Rotation2d(Math.atan2(pose.getX(), pose.getY()));
  }

  public static double getInterpolatedWristAngle(String limelight) {
    return Constants.DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.getInterpolated(
            new InterpolatingDouble(Util.getDistance(limelight)))
        .value;
  }

  public static double getInterpolatedWristAngle() {
    return Constants.DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.getInterpolated(
            new InterpolatingDouble(Util.getDistanceToSpeaker()))
        .value;
  }

  public static boolean isValidShot(String limelight) {
    double dist = Util.getDistance(limelight);
    if (dist > 2.0 && dist < 8.3) {
      return true;
    } else return false;
  }

  public static boolean isValidShot() {
    double dist = Util.getDistanceToSpeaker();
    return dist > 2.25 && dist < 5.25;
  }

  public static double squareValue(double value) {
    return Math.copySign(Math.pow(value, 2), value);
  }

  public static Command pathfindToPose(Pose2d pose) {
    return AutoBuilder.pathfindToPose(
        pose,
        new PathConstraints(5.0, 4.0, Units.degreesToRadians(540.0), Units.degreesToRadians(360.0)),
        0.0,
        0.0);
  }

  public static Pose2d findNearestPose(Pose2d currentPose, Pose2d... otherPoses) {
    return currentPose.nearest(List.of(otherPoses));
  }

  // TODO: Find actual tag positions and ideal offsets
  public static Pose2d findNearestPoseToTrapClimbs(Pose2d currentPose) {
    return currentPose.nearest(TRAP_TAGS);
  }

  public static double maxFiducialAmbiguity(final RawFiducial[] fiducials) {
    double maxAmbiguity = 0.0;
    for (RawFiducial fiducial : fiducials) {
      maxAmbiguity = Math.max(maxAmbiguity, fiducial.ambiguity);
    }
    return maxAmbiguity;
  }
}
