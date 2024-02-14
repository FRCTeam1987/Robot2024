package frc.robot.subsystems.elevator;

import frc.robot.generated.Constants;

public class ElevatorConstants {
  public static final double EXTENSION_KP = 1.6;
  public static final double EXTENSION_KI = 0.0;
  public static final double EXTENSION_KD = 0.0;
  public static final double EXTENSION_KV = 0.3;

  public static final double EXTENSION_MOTION_ACCELERATION = 40000;
  public static final double EXTENSION_CRUISE_VELOCITY = 65000;
  public static final double EXTENSION_ALLOWABLE_ERROR = 500.0;

  public static final double MINIMUM_EXTENSION_LENGTH_INCHES = 0.0;
  public static final double MAXIMUM_EXTENSION_LENGTH_INCHES = 30.5;

  public static final double KRAKEN_TICK_RESOLUTION = Constants.KRAKEN_TICK_RESOLUTION;
  public static final double DRIVE_PULLEY_DIAMETER = 2.256;

  public static final double MOTOR_ROTATIONS_TO_ONE_PULLEY_ROTATION = (54 / 12) * (54 / 18);
  public static final double ROTATIONS_TO_MAXIMUM_EXTENSION =
      (MAXIMUM_EXTENSION_LENGTH_INCHES / DRIVE_PULLEY_DIAMETER);

  public static final double MINIMUM_EXTENSION_MOTOR_TICKS = 0;
  public static final double MAXIMUM_EXTENSION_MOTOR_TICKS =
      KRAKEN_TICK_RESOLUTION
          * MOTOR_ROTATIONS_TO_ONE_PULLEY_ROTATION
          * ROTATIONS_TO_MAXIMUM_EXTENSION;

  public static final double CONVERSION_FACTOR_INCHES_TO_TICKS =
      (MAXIMUM_EXTENSION_MOTOR_TICKS - MINIMUM_EXTENSION_MOTOR_TICKS)
          / (MAXIMUM_EXTENSION_LENGTH_INCHES - MINIMUM_EXTENSION_LENGTH_INCHES);
  public static final double CONVERSION_FACTOR_TICKS_TO_INCHES =
      (MAXIMUM_EXTENSION_LENGTH_INCHES - MINIMUM_EXTENSION_LENGTH_INCHES)
          / (MAXIMUM_EXTENSION_MOTOR_TICKS - MINIMUM_EXTENSION_MOTOR_TICKS);
}
