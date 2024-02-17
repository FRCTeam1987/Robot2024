package frc.robot.subsystems.wrist;

public class WristConstants {

  public static final double WRIST_KP = 1.2;
  public static final double WRIST_KI = 0.0;
  public static final double WRIST_KD = 0.01;
  public static final double WRIST_KV = 0.3;

  public static final double WRIST_ALLOWABLE_ERROR = 0.3;

  public static final double WRIST_CURRENT_LIMIT = 25;

  public static final double WRIST_MOTION_ACCELERATION = 25;
  public static final double WRIST_MOTION_CRUISE_VELOCITY = 70;
  public static final double WRIST_MOTION_JERK = 1200;

  public static final double WRIST_MIN_DEG = 21;
  public static final double WRIST_MAX_DEG = 84;

  public static final double WRIST_MAX_ROT = 3;
  public static final double WRIST_MIN_ROT = 0;

  public static final double CONVERSION_FACTOR_DEGREES_TO_ROTS =
      (WRIST_MAX_ROT - WRIST_MIN_ROT) / (WRIST_MAX_DEG - WRIST_MIN_DEG);
  public static final double CONVERSION_FACTOR_ROTS_TO_DEGREES =
      (WRIST_MAX_DEG - WRIST_MIN_DEG) / (WRIST_MAX_ROT - WRIST_MIN_ROT);
}
