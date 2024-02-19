package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import frc.robot.LimelightHelpers;

public class Constants {
  public static final LimelightHelpers LIMELIGHT = new LimelightHelpers();
  public static final String LIMELIGHT_SCORING = "limelight-scoring";

  public static final double INTAKE_LIMELIGHT_HEIGHT = 0.42; // In meters
  public static final double INTAKE_LIMELIGHT_ANGLE = -13; // In degrees
  public static final double SHOOTER_LIMELIGHT_HEIGHT = 0.13; // In meters
  public static final double SHOOTER_LIMELIGHT_ANGLE = 35; // In degrees

  public static final int SHOOTER_LEADER_ID = 56;
  public static final int SHOOTER_FOLLOWER_ID = 53;
  public static final int SHOOTER_FEEDER_ID = 55;
  public static final int SHOOTER_FEEDER_ID_TEMP = 10;

  public static final int ELEVATOR_LEADER_ID = 60;
  public static final int ELEVATOR_FOLLOWER_ID = 61;

  public static final int INTAKE_TOP_ID = 51; // front / top most roller
  public static final int INTAKE_BOTTOM_ID = 52; // inside / bottom most roller

  public static final int WRIST_ID = 54;

  public static final double INTAKE_COLLECT_VOLTS = 8;  // 6

  public static final double FEEDER_FEEDFWD_VOLTS = 7;  // 4
  public static final double FEEDER_RETRACT_VOLTS = -2;

  public static final double SHOOTER_RPM = 3600;
  public static final double SPIN_RATIO = 0.75;

  public static final InterpolatingTreeMap<Double, Double> DISTANCE_WRIST_ANGLE_MAP =
      new InterpolatingTreeMap<Double, Double>(null, null);

  static {
    // distanceWristAngleMap.put(-3.75, 35.3); // old
    DISTANCE_WRIST_ANGLE_MAP.put(7.8, 39.1);
    DISTANCE_WRIST_ANGLE_MAP.put(8.20, 40.8);
    DISTANCE_WRIST_ANGLE_MAP.put(10.37, 40.8);
    // distanceWristAngleMap.put(11.07, 44.0); // old
    DISTANCE_WRIST_ANGLE_MAP.put(12.89, 40.5);
  }
}
