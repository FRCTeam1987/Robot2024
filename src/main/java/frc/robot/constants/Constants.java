package frc.robot.constants;

import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;

public class Constants {

  public static boolean shouldShuffleboard = false;

  public static double MaxSpeed = 5.0; // 6 meters per second desired top speed
  public static double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public static final double SPEAKER_APRILTAG_HEIGHT = 1.45;
  public static final double TRAP_APRILTAG_HEIGHT = 1.23;
  public static final double AMP_APRILTAG_HEIGHT = 0.535;

  public static final double translationXSlewRate = 4.0;
  public static final double translationYSlewRate = 4.0;
  public static final double rotationSlewRate = 8.0;

  public static final int SHOOTER_LEADER_ID = 56;
  public static final int SHOOTER_FOLLOWER_ID = 53;
  public static final int SHOOTER_FEEDER_ID = 55;
  public static final int SHOOTER_FEEDER_ID_TEMP = 10;

  public static final int SHOOTER_AMP_RPM = 650;
  public static final double ELEVATOR_AMP_HEIGHT = 6.2;
  public static final double WRIST_AMP_DEGREES = 110.0;

  public static final double ELEVATOR_TRAP_HEIGHT = 28.0;
  public static final double ELEVATOR_TRAP_COLLAPSED_HEIGHT = 0.05;

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

  public static final double FEEDER_FEEDFWD_VOLTS = 4; // 6 // 4
  public static final double FEEDER_SHOOT_VOLTS = 5; // 4
  public static final double FEEDER_RETRACT_VOLTS = -2;

  public static final double SHOOTER_RPM = 3500;
  public static final double SHOOTER_RPM_CLOSERANGE = 3500; // NEEDS to bee smaller
  public static final double SHOOTER_IDLE_RPM = 2500;
  public static final double SHOOTER_IDLE_CLOSERANGE_RPM = 3250; // NEEDS to bee smaller
  public static final double SPIN_RATIO = 0.75;

  public static final double CLIMBER_NOMINAL_VOLTAGE = 8;
  public static final double CLIMBER_MAINTAIN_VOLTAGE = 0.15;
  public static final double CLIMBER_CUTOFF_AMPERAGE = 50;

  public static final InterpolatingTreeMap<
          InterpolatingDouble,
          InterpolatingDouble> // TODO Update Limelight Constants with new position
      DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR = // (Meters, Wrist Degrees)
      new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

  //   public static final InterpolatingTreeMap<
  //           InterpolatingDouble,
  //           InterpolatingDouble> // TODO Update Limelight Constants with new position
  //       DISTANCE_WRIST_ANGLE_MAP_ELEVATOR = // (Meters, Wrist Degrees)
  //       new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

  static {
    // DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.put(new InterpolatingDouble(0.9), new
    // InterpolatingDouble(36.0));
    DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.put(
        new InterpolatingDouble(-7.22), new InterpolatingDouble(32.7));
    DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.put(
        new InterpolatingDouble(-10.6), new InterpolatingDouble(30.0));
    DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.put(
        new InterpolatingDouble(-12.5), new InterpolatingDouble(29.0));
    DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.put(
        new InterpolatingDouble(-14.3), new InterpolatingDouble(28.3));
    DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.put(
        new InterpolatingDouble(-15.3), new InterpolatingDouble(27.1));

    // DISTANCE_WRIST_ANGLE_MAP_ELEVATOR.put(
    //     new InterpolatingDouble(1.99), new InterpolatingDouble(34.0));
    // DISTANCE_WRIST_ANGLE_MAP_ELEVATOR.put(
    //     new InterpolatingDouble(1.75), new InterpolatingDouble(36.0));
    // DISTANCE_WRIST_ANGLE_MAP_ELEVATOR.put(
    //     new InterpolatingDouble(1.3), new InterpolatingDouble(42.0));
    // DISTANCE_WRIST_ANGLE_MAP_ELEVATOR.put(
    //     new InterpolatingDouble(0.93), new InterpolatingDouble(49.0));
    // DISTANCE_WRIST_ANGLE_MAP_ELEVATOR.put(
    //     new InterpolatingDouble(0.85), new InterpolatingDouble(49.0));
    // DISTANCE_WRIST_ANGLE_MAP.put(new InterpolatingDouble(3.2), new InterpolatingDouble(28.4));
  }
}
