package frc.robot.constants;

import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;

public class Constants {

  public static final boolean shouldShuffleboard = false;

  public static final double MaxSpeed =
      DriveConstants.SPEED_AT_12_VOLTS_MPS; // 6 meters per second desired top speed
  public static final double MaxAngularRate = Math.toRadians(540.00);
  // 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public static final double SPEAKER_APRILTAG_HEIGHT = 1.458;
  public static final double TRAP_APRILTAG_HEIGHT = 1.315;
  public static final double AMP_APRILTAG_HEIGHT = 1.35; // 0.535

  public static final double translationXSlewRate = 8.0;
  public static final double translationYSlewRate = 8.0;
  public static final double rotationSlewRate = 8.0;

  public static final int SHOOTER_LEADER_ID = 56;
  public static final int SHOOTER_FOLLOWER_ID = 53;
  public static final int SHOOTER_FEEDER_ID = 55;
  public static final int SHOOTER_FEEDER_ID_TEMP = 10;

  public static final double ELEVATOR_AMP_HEIGHT = 6.2;
  public static final double WRIST_AMP_DEGREES = 110.0;

  public static final double ELEVATOR_TRAP_HEIGHT = 28.0;
  public static final double ELEVATOR_TRAP_COLLAPSED_HEIGHT = 5.9;

  public static final int LEFT_CANDLE = 28;
  public static final int RIGHT_CANDLE = 29;

  public static final int ELEVATOR_LEADER_ID = 60;
  public static final int ELEVATOR_FOLLOWER_ID = 61;

  public static final int CLIMB_LEFT = 58;
  public static final int CLIMB_RIGHT = 57;

  public static final int INTAKE_TOP_ID = 51; // front / top most roller
  public static final int INTAKE_BOTTOM_ID = 52; // inside / bottom most roller

  public static final int WRIST_ID = 54;

  public static final double INTAKE_COLLECT_VOLTS = -6; // 6

  public static class Trap {
    public static final double TRAP_DEBOUNCE_TIME = 0.06;
    public static final double TRAP_ELEVATOR_HEIGHT = 29.5;
    public static final double TRAP_WRIST_DEGREES = 112.5;
    public static final double TRAP_RPM_SPEED = 525;
  }

  public static class Wrist {
    public static final double WRIST_KP = 4.8;
    public static final double WRIST_KI = 0.0;
    public static final double WRIST_KD = 0.01;
    public static final double WRIST_KV = 0.03;

    public static final double WRIST_ALLOWABLE_ERROR = 0.0014;

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

  public static class Shooter {
    public static final double FEEDER_FEEDFWD_VOLTS = 4; // 6 // 4
    public static final double FEEDER_FEEDFWD_VOLTS_AGRESSIVE = 8; // 6 // 4
    public static final double FEEDER_SHOOT_VOLTS = 5; // 4
    public static final double FEEDER_RETRACT_VOLTS = -2;

    public static final double SHOOTER_RPM = 4000;
    public static final double SHOOTER_RPM_CLOSERANGE = 3500; // NEEDS to be smaller
    public static final double SHOOTER_LOB_RPM = 3000; // NEEDS to be smaller
    public static final double SHOOTER_IDLE_RPM = 500; // 2500
    public static final double SHOOTER_IDLE_CLOSERANGE_RPM = 2500; // NEEDS to be smaller
    public static final double SPIN_RATIO = 0.75;
    public static final int SHOOTER_AMP_RPM = 550;
  }

  public static class Elevator {

    public static final double EXTENSION_KP = 2.5;
    public static final double EXTENSION_KI = 0.0;
    public static final double EXTENSION_KD = 0.1;
    public static final double EXTENSION_KV = 0.15;

    public static final double EXTENSION_KP_1 = 8.0;
    public static final double EXTENSION_KI_1 = 0.6;
    public static final double EXTENSION_KD_1 = 0.1;
    public static final double EXTENSION_KV_1 = 0.45;

    public static final double EXTENSION_MOTION_ACCELERATION = 45000;
    public static final double EXTENSION_CRUISE_VELOCITY = 65000;
    public static final double EXTENSION_ALLOWABLE_ERROR = 5.0;

    public static final double EXTENSION_CURRENT_LIMIT = 90.0;

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

  public static class Climber {
    public static final double EXTENSION_CURRENT_LIMIT = 23.0;
    public static final double CLIMBER_NOMINAL_VOLTAGE = 10;
    public static final double CLIMBER_MAINTAIN_VOLTAGE = 0.35;
    public static final double CLIMBER_CUTOFF_AMPERAGE = 65;
  }

  public static final InterpolatingTreeMap<
          InterpolatingDouble,
          InterpolatingDouble> // TODO Update Limelight Constants with new position
      DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR = // (Meters, Wrist Degrees)
      new InterpolatingTreeMap<>();

  public static final InterpolatingTreeMap<
          InterpolatingDouble,
          InterpolatingDouble> // TODO Update Limelight Constants with new position
      PITCH_TO_DISTANCE_RELATIVE_SPEAKER = // (Meters, Wrist Degrees)
      new InterpolatingTreeMap<>();

  public static final InterpolatingTreeMap<
          InterpolatingDouble,
          InterpolatingDouble> // TODO Update Limelight Constants with new position
      DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER = // (Meters, Wrist Degrees)
      new InterpolatingTreeMap<>();

  static {
    PITCH_TO_DISTANCE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(-8.02), new InterpolatingDouble(2.13));
    PITCH_TO_DISTANCE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(-10.69), new InterpolatingDouble(2.361));
    PITCH_TO_DISTANCE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(-12.14), new InterpolatingDouble(2.502));
    PITCH_TO_DISTANCE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(-14.02), new InterpolatingDouble(2.724));
    PITCH_TO_DISTANCE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(-16.97), new InterpolatingDouble(3.152));
    PITCH_TO_DISTANCE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(-18.10), new InterpolatingDouble(3.348));
    PITCH_TO_DISTANCE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(-20.15), new InterpolatingDouble(3.740));
    PITCH_TO_DISTANCE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(-21.2), new InterpolatingDouble(4.020));
    PITCH_TO_DISTANCE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(-22.0), new InterpolatingDouble(4.228));

    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(2.334), new InterpolatingDouble(36.05));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(2.571), new InterpolatingDouble(35.66));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(2.798), new InterpolatingDouble(34.48));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(2.982), new InterpolatingDouble(33.96));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(3.087), new InterpolatingDouble(32.13));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(3.203), new InterpolatingDouble(31.08));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(3.383), new InterpolatingDouble(30.62));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(3.695), new InterpolatingDouble(28.92));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(4.08), new InterpolatingDouble(27.6));

    // DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.put(new InterpolatingDouble(0.9), new

    DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.put(
        new InterpolatingDouble(-8.3), new InterpolatingDouble(33.0)); // 10 ft away
    DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.put(
        new InterpolatingDouble(-9.7), new InterpolatingDouble(32.0)); // 10 ft away
    DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.put(
        new InterpolatingDouble(-10.5), new InterpolatingDouble(31.0)); // 10 ft away
    DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.put(
        new InterpolatingDouble(-11.2), new InterpolatingDouble(30.5)); // 11 ft away
    DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.put(
        new InterpolatingDouble(-12.5), new InterpolatingDouble(29.0)); // 12 ft away
    DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.put(
        new InterpolatingDouble(-14.3), new InterpolatingDouble(28.5)); //
  }
}
