/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.LimelightHelpers.RawFiducial;
import java.util.List;

public class Util {
  public static final AprilTagFieldLayout field =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  public static Pose2d TAG_4_POSE; // RED ALLIANCE SPEAKER CENTER
  public static Pose2d TAG_7_POSE; // BLUE ALLIANCE SPEAKER CENTER
  public static Pose3d TAG_5_POSE; // RED AMP
  public static Pose3d TAG_6_POSE; // BLUE AMP
  public static Pose2d MANUAL_RED_AMP; // RED AMP
  public static Pose2d MANUAL_BLUE_AMP; // BLUE AMP
  public static Pose2d MANUAL_RED_LOB; // RED AMP
  public static Pose2d MANUAL_BLUE_LOB; // BLUE AMP
  public static Pose2d TAG_LOB_BLUE;
  public static Pose2d TAG_LOB_RED;
  public static List<Pose2d> TRAP_TAGS;
  public static List<Pose2d> AMP_TAGS;
  public static DriverStation.Alliance alliance;
  public static final double DEADBAND = 0.05;

  public Util() {
    field
        .getTagPose(4)
        .ifPresent(
            pose -> {
              Util.TAG_4_POSE =
                  pose.toPose2d(); // .transformBy(new Transform3d(0, 0.565, 0, new Rotation3d()));
            });
    field
        .getTagPose(7)
        .ifPresent(
            pose -> {
              Util.TAG_7_POSE =
                  pose.toPose2d(); // .transformBy(new Transform3d(0, 0.565, 0, new Rotation3d()));
            });
    MANUAL_RED_AMP = new Pose2d(14.7, 7.42, new Rotation2d(Math.toRadians(90.0)));
    MANUAL_BLUE_AMP = new Pose2d(1.84, 7.42, new Rotation2d(Math.toRadians(90.0)));
    MANUAL_RED_LOB = new Pose2d(14.7, 6.5, new Rotation2d(Math.toRadians(90.0)));
    MANUAL_BLUE_LOB = new Pose2d(1.84, 6.5, new Rotation2d(Math.toRadians(90.0)));
    TRAP_TAGS = List.of();
    DriverStation.getAlliance().ifPresent(ouralliance -> alliance = ouralliance);
  }

  public static Pose2d getAllianceSpeaker() {
    // return alliance == DriverStation.Alliance.Blue ? TAG_7_POSE : TAG_4_POSE;
    return CommandSwerveDrivetrain.getAlliance() == DriverStation.Alliance.Blue
        ? TAG_7_POSE
        : TAG_4_POSE;
  }

  public static Pose2d getAllianceAmp() {
    // return alliance == DriverStation.Alliance.Blue ? TAG_7_POSE : TAG_4_POSE;
    return CommandSwerveDrivetrain.getAlliance() == DriverStation.Alliance.Blue
        ? MANUAL_BLUE_AMP
        : MANUAL_RED_AMP;
  }

  public static Pose2d getAllianceLob() {
    // return alliance == DriverStation.Alliance.Blue ? TAG_7_POSE : TAG_4_POSE;
    return CommandSwerveDrivetrain.getAlliance() == DriverStation.Alliance.Blue
        ? MANUAL_BLUE_LOB
        : MANUAL_RED_LOB;
  }

  public static Rotation2d getRotationToAllianceLob(Pose2d opose) {
    Transform2d delta = new Transform2d(Util.getAllianceLob().getTranslation().minus(opose.getTranslation()), new Rotation2d());
    return new Rotation2d(Math.atan2(delta.getY(), delta.getX())).plus(Rotation2d.fromDegrees(180.0));
  }

  public static double getShooterSpeedFromDistanceForLob(double distance) {
    return Constants.DISTANCE_TO_LOB_RPM.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public static double getDistanceToAllianceLob(Pose2d opose) {
    return RobotContainer.get().getPose()
        .getTranslation()
        .getDistance(
            getAllianceLob().getTranslation());
  }
  public static final Pose2d BLUE_AUTO_SOURCE_SHOOTING_POSE = new Pose2d(3.57, 2.99, Rotation2d.fromDegrees(-35.0));
  public static final Pose2d RED_AUTO_SOURCE_SHOOTING_POSE = new Pose2d(13.02, 2.99, Rotation2d.fromDegrees(-145.0));
  public static Command PathFindToAutoSourceShot() {
    return new ConditionalCommand(
      Util.pathfindToPose(BLUE_AUTO_SOURCE_SHOOTING_POSE),
      Util.pathfindToPose(RED_AUTO_SOURCE_SHOOTING_POSE),
      () -> RobotContainer.DRIVETRAIN.getAlliance().equals(Alliance.Blue)
    );
  }

  public static boolean isWithinTolerance(
      double currentValue, double targetValue, double tolerance) {
    return Math.abs(currentValue - targetValue) <= tolerance;
  }

  public static boolean canSeeTarget(String limelight) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight).tagCount > 0;
  }

  public static void setupUtil() {}

  // TODO flex on alliance tag pose
  public static double getDistanceToSpeaker() {
    return RobotContainer.get().getPose()
        .getTranslation()
        .getDistance(
            getAllianceSpeaker().getTranslation());
  }

  public static double getDistanceToAmp() {
    return RobotContainer.get().getPose()
        .getTranslation()
        .getDistance(
            getAllianceAmp().getTranslation());
  }

      // System.out.println("SPEAKER: " + Util.getAllianceSpeaker());
    // System.out.println("SUBTRACT: " + delta);
    // System.out.println(delta.getAngle());

  public static Rotation2d getRotationToAllianceSpeaker(Pose2d opose) {
    //Pose2d newpose = new Pose2d(opose.getTranslation(), new Rotation2d(90.0));
    Transform2d delta = new Transform2d(Util.getAllianceSpeaker().getTranslation().minus(opose.getTranslation()), new Rotation2d());
    return new Rotation2d(Math.atan2(delta.getY(), delta.getX())).plus(Rotation2d.fromDegrees(180.0));
  }

  public static double getInterpolatedWristAngleSpeaker() {
    return Constants.DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.getInterpolated(
            new InterpolatingDouble(Util.getDistanceToSpeaker()))
        .value;
  }

  public static boolean isValidShot() {
    double dist = Util.getDistanceToSpeaker();
    return (dist > 2.25 && dist < 5.25);
  }

  public static boolean isValidShotAmpInclusive() {
    double dist = Util.getDistanceToSpeaker();
    double ampDist = Util.getDistanceToAmp();
    return (dist > 2.25 && dist < 5.25) || ampDist < 4.00;
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

  public static double speakerTagCount(final RawFiducial[] fiducials) {
    int tagCount = 0;
    for (RawFiducial fiducial : fiducials) {
      if (Constants.Vision.SPEAKER_TAG_IDS.contains(fiducial.id)) {
        tagCount++;
      }
    }
    return tagCount;
  }

  public static boolean isPointedAtSpeaker(CommandSwerveDrivetrain drivetrain) {
    Pose2d current = drivetrain.getPose();
    return Util.isWithinTolerance(
        current.getRotation().getDegrees(),
        Util.getRotationToAllianceSpeaker(current).getDegrees(),
        3.0);
  }

  public static boolean isPointedAtLob(CommandSwerveDrivetrain drivetrain) {
    Pose2d current = drivetrain.getPose();
    return Util.isWithinTolerance(
        current.getRotation().getDegrees(),
        Util.getRotationToAllianceLob(current).getDegrees(),
        3.0);
  }
}
