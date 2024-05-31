package frc.robot.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.RectanglePoseArea;
import java.util.Arrays;
import java.util.List;

public class Constants {

  public static final boolean shouldShuffleboard = false;

  public static final double MaxSpeed =
      DriveConstants.kSpeedAt12VoltsMps; // 6 meters per second desired top speed
  public static final double MaxAngularRate = Math.toRadians(540.00);
  // 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public static final double SPEAKER_APRILTAG_HEIGHT = 1.458;
  public static final double TRAP_APRILTAG_HEIGHT = 1.315;
  public static final double AMP_APRILTAG_HEIGHT = 1.35; // 0.535

  public static final double translationSlewRate = 2.5;
  //   public static final double rotationSlewRate = 10;

  public static final int SHOOTER_LEADER_ID = 53;
  public static final int SHOOTER_FOLLOWER_ID = 56;
  public static final int SHOOTER_FEEDER_ID = 55;
  public static final int SHOOTER_FEEDER_ID_TEMP = 10;

  public static final double FWD_ELEVATOR_AMP_HEIGHT = 6.2;
  public static final double FWD_WRIST_AMP_DEGREES = 110.0;

  public static final double REV_ELEVATOR_AMP_HEIGHT = 28.5;
  public static final double REV_WRIST_AMP_DEGREES = 8.0;
  public static final double REV_FEEDER_VOLTAGE = -7.0;

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
  public static final double INTAKE_COLLECT_VOLTS_MANUAL = -8; // 6 //-9
  public static final double INTAKE_RPM = -5000; // 6 //-9

  public static class Trap {
    public static final double TRAP_DEBOUNCE_TIME = 0.06;
    public static final double TRAP_ELEVATOR_HEIGHT = 29.5;
    public static final double TRAP_ELEVATOR_HEIGHT_MIDWAY = 24.0;
    public static final double TRAP_WRIST_DEGREES = 119.0;
    public static final double TRAP_WRIST_DEGREES_MIDWAY = 85.0;
    public static final double TRAP_RPM_SPEED =
        375; //225
  }

  public static class Wrist {
    public static final int PROXIMITY_SENSOR_LEFT_ID = 9;
    public static final int PROXIMITY_SENSOR_RIGHT_ID = 8; // TODO: Update left and right

    public static final double WRIST_KP = 250.0; // 200
    public static final double WRIST_KI = 0.0;
    public static final double WRIST_KD = 0.01;
    public static final double WRIST_KV = 0.03;

    public static final double WRIST_ALLOWABLE_ERROR = 0.0014;

    public static final double WRIST_CURRENT_LIMIT = 25;

    public static final double WRIST_MOTION_ACCELERATION = 100;
    public static final double WRIST_MOTION_CRUISE_VELOCITY = 8600;
    public static final double WRIST_MOTION_JERK = 0;

    public static final double WRIST_MIN_DEG = 7;
    public static final double WRIST_MAX_DEG = 120; // temp 115;

    public static final double WRIST_MAX_ROT = 3;
    public static final double WRIST_MIN_ROT = 0;

    public static final double CONVERSION_FACTOR_DEGREES_TO_ROTS =
        (WRIST_MAX_ROT - WRIST_MIN_ROT) / (WRIST_MAX_DEG - WRIST_MIN_DEG);
    public static final double CONVERSION_FACTOR_ROTS_TO_DEGREES =
        (WRIST_MAX_DEG - WRIST_MIN_DEG) / (WRIST_MAX_ROT - WRIST_MIN_ROT);

    public static final double INITIAL_ANGLE_DEGREES = 7.0;
  }

  public static class Shooter {
    public static final double FEEDER_FEEDFWD_VOLTS = 4.25; //4
    public static final double FEEDER_FEEDFWD_VOLTS_AGRESSIVE = 6; // 6 // 4
    public static final double FEEDER_SHOOT_VOLTS = 8; // 4
    public static final double FEEDER_RETRACT_VOLTS = -2;
    public static final double FEEDER_AUTO_VOLTS = 8.0;

    public static final double SHOOTER_RPM = 4500;
    public static final double SHOOTER_RPM_CLOSERANGE = 3500; // NEEDS to be smaller
    public static final double SHOOTER_LOB_RPM = 3000; // NEEDS to be smaller
    public static final double SHOOTER_IDLE_SHOOTING_RPM = 3000; // NEEDS to be smaller
    public static final double SHOOTER_IDLE_RPM = 2500; // 2500
    public static final double SHOOTER_IDLE_RPM_CLOSE = 4200; // 2500
    public static final double SHOOTER_IDLE_CLOSERANGE_RPM = 2500; // NEEDS to be smaller
    public static final double SPIN_RATIO = 0.66; // 0.85
    public static final double ANTI_SPIN_RATIO = 1.34; // 0.85
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
    public static final double EXTENSION_ALLOWABLE_ERROR = 0.375;

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

  public static class Climb {
    public static final double CLIMB_PULLDOWN_HEIGHT = 6.75;
    public static final double CLIMB_LEVEL_HEIGHT = 11.0;
    public static final double CLIMB_START_HEIGHT = 28.0;
  }

  public static class Climber {
    public static final double EXTENSION_CURRENT_LIMIT = 23.0;
    public static final double CLIMBER_NOMINAL_VOLTAGE = 10;
    public static final double CLIMBER_MAINTAIN_VOLTAGE = 0.35;
    public static final double CLIMBER_CUTOFF_AMPERAGE = 65;
  }

  //   public static final InterpolatingTreeMap<
  //           InterpolatingDouble,
  //           InterpolatingDouble> // TODO Update Limelight Constants with new position
  //       DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR = // (Meters, Wrist Degrees)
  //       new InterpolatingTreeMap<>();
  public static final InterpolatingTreeMap<
          InterpolatingDouble,
          InterpolatingDouble> // TODO Update Limelight Constants with new position
      DISTANCE_TO_LOB_RPM = // (Meters, Wrist Degrees)
      new InterpolatingTreeMap<>();
  public static final InterpolatingTreeMap<
          InterpolatingDouble,
          InterpolatingDouble> // TODO Update Limelight Constants with new position
      DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER = // (Meters, Wrist Degrees)
      new InterpolatingTreeMap<>();

  static {
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(2.334), new InterpolatingDouble(35.50));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(2.40), new InterpolatingDouble(35.10));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(2.50), new InterpolatingDouble(34.23));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(2.69), new InterpolatingDouble(32.87));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(2.87), new InterpolatingDouble(31.69));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(3.03), new InterpolatingDouble(30.24));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(3.207), new InterpolatingDouble(28.86));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(3.388),
        new InterpolatingDouble(28.21)); // Closer values may require a slower shooter RPM Speed
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(3.70), new InterpolatingDouble(27.08));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(4.03), new InterpolatingDouble(25.63));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(4.23), new InterpolatingDouble(25.07));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(4.46), new InterpolatingDouble(24.65));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(4.69), new InterpolatingDouble(24.35));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(4.99), new InterpolatingDouble(23.21));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(5.21), new InterpolatingDouble(22.81));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(5.40), new InterpolatingDouble(22.92));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(5.69), new InterpolatingDouble(21.63));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(5.89), new InterpolatingDouble(21.47));
    DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.put(
        new InterpolatingDouble(6.15), new InterpolatingDouble(21.45));

    DISTANCE_TO_LOB_RPM.put(new InterpolatingDouble(10.28), new InterpolatingDouble(3000.0));
    DISTANCE_TO_LOB_RPM.put(new InterpolatingDouble(9.5), new InterpolatingDouble(2750.0));
    DISTANCE_TO_LOB_RPM.put(new InterpolatingDouble(8.6), new InterpolatingDouble(2700.0));
    DISTANCE_TO_LOB_RPM.put(new InterpolatingDouble(8.3), new InterpolatingDouble(2620.0));
    DISTANCE_TO_LOB_RPM.put(new InterpolatingDouble(8.0), new InterpolatingDouble(2580.0));
    DISTANCE_TO_LOB_RPM.put(new InterpolatingDouble(7.88), new InterpolatingDouble(2440.0));
  }

  public static class Vision {
    // public static final String SPEAKER_RIGHT_LOW_LIMELIGHT = "limelight-rightlo";
    public static final String LEFT_LOW = "limelight-leftlo";
    public static final String RIGHT_LIMELIGHT = "limelight-right";
    public static final String LEFT_LIMELIGHT = "limelight-left";
    public static final String RIGHT_LOW = "limelight-rightlo";
    public static final List<String> LL3GS = Arrays.asList(LEFT_LOW, RIGHT_LOW);
    public static final List<String> LL3S = Arrays.asList(); // RIGHT_LIMELIGHT, LEFT_LIMELIGHT
    public static final List<Integer> SPEAKER_TAG_IDS = Arrays.asList(3, 4, 7, 8);
    public static final double MAX_DISTANCE_SCALING = 5.5;
    public static final RectanglePoseArea fieldBoundary =
        new RectanglePoseArea(new Translation2d(0, 0), new Translation2d(16.541, 8.211));
    public static final double maxMutiTagDistToAccept = Units.feetToMeters(25.0); // 15.0
    public static final double maxTagDistToTrust = Units.feetToMeters(15.0); // 15.0
    public static final double maxSingleTagDistanceToAccept = Units.feetToMeters(10.0);
    public static final Vector<N3> absoluteTrustVector = VecBuilder.fill(.2, .2, 1);
  }
}
