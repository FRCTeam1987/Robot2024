package frc.robot.subsystems.wrist;

public class WristConstants {

  public static final double WRIST_KP = 4.8;
  public static final double WRIST_KI = 0.0;
  public static final double WRIST_KD = 0.01;
  public static final double WRIST_KV = 0.03;

  public static final double WRIST_ALLOWABLE_ERROR = 0.3;

  public static final double WRIST_CURRENT_LIMIT = 25;

  public static final double WRIST_MOTION_ACCELERATION = 100;
  public static final double WRIST_MOTION_CRUISE_VELOCITY = 8600;
  public static final double WRIST_MOTION_JERK = 0;

  public static final double WRIST_MIN_DEG = 7;
  public static final double WRIST_MAX_DEG = 114; // temp 115;

  public static final double WRIST_MAX_ROT = 3;
  public static final double WRIST_MIN_ROT = 0;

  public static final double CONVERSION_FACTOR_DEGREES_TO_ROTS =
      (WRIST_MAX_ROT - WRIST_MIN_ROT) / (WRIST_MAX_DEG - WRIST_MIN_DEG);
  public static final double CONVERSION_FACTOR_ROTS_TO_DEGREES =
      (WRIST_MAX_DEG - WRIST_MIN_DEG) / (WRIST_MAX_ROT - WRIST_MIN_ROT);

  public static final double INITIAL_ANGLE_DEGREES = 7.0;
}
