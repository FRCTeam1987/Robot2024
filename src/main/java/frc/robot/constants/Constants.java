package frc.robot.constants;

import frc.robot.LimelightHelpers;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;

public class Constants {
  public static final LimelightHelpers LIMELIGHT = new LimelightHelpers();
  public static final String LIMELIGHT_SCORING = "limelight-scoring";
  public static final double SPEAKER_APRILTAG_HEIGHT = 1.45;
  public static final double TRAP_APRILTAG_HEIGHT = 1.23;

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

  public static final double INTAKE_COLLECT_VOLTS = -6; // 6

  public static final double FEEDER_FEEDFWD_VOLTS = 2; // 6 // 4
    public static final double FEEDER_SHOOT_VOLTS = 5 ; // 4
  public static final double FEEDER_RETRACT_VOLTS = -2;

  public static final double SHOOTER_RPM = 5500;
  public static final double SPIN_RATIO = 0.75;

  public static final InterpolatingTreeMap<
          InterpolatingDouble,
          InterpolatingDouble> // TODO Update Limelight Constants with new position
      DISTANCE_WRIST_ANGLE_MAP = // (Meters, Wrist Degrees)
      new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

  static {
    DISTANCE_WRIST_ANGLE_MAP.put(new InterpolatingDouble(2.26), new InterpolatingDouble(36.0));
    DISTANCE_WRIST_ANGLE_MAP.put(new InterpolatingDouble(2.67), new InterpolatingDouble(30.5));
    DISTANCE_WRIST_ANGLE_MAP.put(new InterpolatingDouble(3.00), new InterpolatingDouble(30.3));
    DISTANCE_WRIST_ANGLE_MAP.put(new InterpolatingDouble(3.00), new InterpolatingDouble(30.3));
    DISTANCE_WRIST_ANGLE_MAP.put(new InterpolatingDouble(3.2), new InterpolatingDouble(28.4));
  }
}
