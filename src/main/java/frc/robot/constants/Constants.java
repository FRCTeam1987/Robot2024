package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public class Constants {
  public static final int SHOOTER_LEADER_ID = 56;
  public static final int SHOOTER_FOLLOWER_ID = 53;
  public static final int SHOOTER_FEEDER_ID = 55;

  public static final int ELEVATOR_LEADER_ID = 60;
  public static final int ELEVATOR_FOLLOWER_ID = 61;

  public static final int INTAKE_TOP_ID = 51; // front / top most roller
  public static final int INTAKE_BOTTOM_ID = 52; // inside / bottom most roller

  public static final int WRIST_ID = 54;

  public static final double MAXIMUM_SPEED = 6; // Meters per second

  public static final double INTAKE_LIMELIGHT_HEIGHT = 0.42; // In meters
  public static final double INTAKE_LIMELIGHT_ANGLE = -13; // In degrees
  public static final double SHOOTER_LIMELIGHT_HEIGHT = 0.13; // In meters
  public static final double SHOOTER_LIMELIGHT_ANGLE = 35; // In degrees

  public static final double INTAKE_COLLECT_VOLTS = 6;
  public static final double FEEDER_FEEDFWD_VOLTS = 12;
  public static final double FEEDER_RETRACT_VOLTS = -2;

  public static final InterpolatingTreeMap<Double, Double> distanceWristAngleMap =
      new InterpolatingTreeMap<Double, Double>(null, null);

  static {
    distanceWristAngleMap.put(-3.75, 35.3);
    distanceWristAngleMap.put(-0.79, 37.2);
    distanceWristAngleMap.put(2.07, 38.9);
    distanceWristAngleMap.put(6.7,42.0);
    distanceWristAngleMap.put(11.07, 44.0);
  }
}
