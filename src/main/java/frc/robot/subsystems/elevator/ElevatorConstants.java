package frc.robot.subsystems.elevator;

public class ElevatorConstants {
  public static final double EXTENSION_KP = 0.2;
  public static final double EXTENSION_KI = 0.0;
  public static final double EXTENSION_KD = 0.1;
  public static final double EXTENSION_KV = 0.6;

  public static final double EXTENSION_MOTION_ACCELERATION = 30;
  public static final double EXTENSION_CRUISE_VELOCITY = 1200;
  public static final double EXTENSION_JERK = 1200;
  public static final double EXTENSION_ALLOWABLE_ERROR = 5.0;

  public static final double EXTENSION_CURRENT_LIMIT = 25.0;

  public static final double MINIMUM_EXTENSION_LENGTH_INCHES = 0.0;
  public static final double MAXIMUM_EXTENSION_LENGTH_INCHES = 30.5;

  public static final double MINIMUM_EXTENSION_ROTATIONS = 0.0;
  public static final double MAXIMUM_EXTENSION_ROTATIONS = 55.871;

  public static final double CONVERSION_FACTOR_INCHES_TO_TICKS =
      (MAXIMUM_EXTENSION_ROTATIONS - MINIMUM_EXTENSION_ROTATIONS)
          / (MAXIMUM_EXTENSION_LENGTH_INCHES - MINIMUM_EXTENSION_LENGTH_INCHES);
  public static final double CONVERSION_FACTOR_TICKS_TO_INCHES =
      (MAXIMUM_EXTENSION_LENGTH_INCHES - MINIMUM_EXTENSION_LENGTH_INCHES)
          / (MAXIMUM_EXTENSION_ROTATIONS - MINIMUM_EXTENSION_ROTATIONS);
}
