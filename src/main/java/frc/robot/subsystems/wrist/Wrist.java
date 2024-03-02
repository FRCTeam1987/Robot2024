package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private final TalonFX WRIST_MOTOR;
  private final ShuffleboardTab WRIST_TAB = Shuffleboard.getTab("WRIST");
  private double IncrementValue = 0.0;

  // Constructor
  public Wrist(final int wristMotorID) {
    WRIST_MOTOR = new TalonFX(wristMotorID, "rio");
    final TalonFXConfiguration WRIST_CONFIG = new TalonFXConfiguration();

    WRIST_CONFIG.Slot0.kP = 500.0; // WristConstants.WRIST_KP;
    WRIST_CONFIG.Slot0.kI = WristConstants.WRIST_KI;
    WRIST_CONFIG.Slot0.kD = WristConstants.WRIST_KD;
    WRIST_CONFIG.Slot0.kV = WristConstants.WRIST_KV;
    // WRIST_CONFIG.Slot0.kS = 0.4;

    // final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    // currentLimits.SupplyCurrentLimit = 20; // Limit to 1 amps
    // currentLimits.SupplyCurrentThreshold = 25; // If we exceed 4 amps
    // currentLimits.SupplyTimeThreshold = 0.2; // For at least 1 second
    // currentLimits.SupplyCurrentLimitEnable = true; // And enable it

    // currentLimits.StatorCurrentLimit = 20; // Limit stator to 20 amps
    // currentLimits.StatorCurrentLimitEnable = true; // And enable it

    // WRIST_CONFIG.CurrentLimits = currentLimits;

    WRIST_CONFIG.CurrentLimits.StatorCurrentLimit = WristConstants.WRIST_CURRENT_LIMIT;
    // WRIST_CONFIG.CurrentLimits.SupplyCurrentLimit = WristConstants.WRIST_CURRENT_LIMIT;
    // WRIST_CONFIG.SupplyCurrentT
    // WRIST_CONFIG.CurrentLimits.SupplyCurrentLimitEnable  = true;
    WRIST_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

    WRIST_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 50;
    // WristConstants.WRIST_MOTION_CRUISE_VELOCITY;
    WRIST_CONFIG.MotionMagic.MotionMagicAcceleration =
        WRIST_CONFIG.MotionMagic.MotionMagicCruiseVelocity;
    // WristConstants.WRIST_MOTION_ACCELERATION;
    // WRIST_CONFIG.MotionMagic.MotionMagicJerk = 10;
    WRIST_CONFIG.Feedback.SensorToMechanismRatio = (100 / 10) * (36 / 18); // 22:1
    // WristConstants.WRIST_MOTION_JERK;

    WRIST_MOTOR.getConfigurator().apply(WRIST_CONFIG);
    WRIST_MOTOR.setPosition(WristConstants.INITIAL_ANGLE_DEGREES / 360.0);
    WRIST_MOTOR.setNeutralMode(NeutralModeValue.Brake);

    setupShuffleboard();
  }

  public void goHome() {
    setDegrees(WristConstants.INITIAL_ANGLE_DEGREES);
  }

  public void zeroSensor() {
    WRIST_MOTOR.setPosition(WristConstants.INITIAL_ANGLE_DEGREES / 360.0);
  }

  public boolean isAtSetpoint() {
    return WRIST_MOTOR.getClosedLoopError().getValueAsDouble()
        < WristConstants.WRIST_ALLOWABLE_ERROR;
  }

  public double getDegrees() {
    // initial angle irl is 13 degrees
    // 44 degrees at 1.67 rotations
    // Reduction is 18T to 36T pulley, 10T to 100T gear
    return WRIST_MOTOR.getPosition().getValueAsDouble() * 360.0;
    // * WristConstants.CONVERSION_FACTOR_ROTS_TO_DEGREES;
  }

  public void setDegrees(double degrees) {
    degrees = degrees + IncrementValue;

    if (degrees > WristConstants.WRIST_MAX_DEG || degrees < WristConstants.WRIST_MIN_DEG) {
      System.out.println("Out of Wrist Range!");
      return;
    } else {
      double arbFF = 0.4 * Math.sin(Math.toRadians(90.0 - degrees));
      WRIST_MOTOR.setControl(
          new MotionMagicVoltage(degrees / 360.0, true, arbFF, 0, false, false, false));
    }
  }

  public void incrementWrist(double IncrementAmount) {
    IncrementValue = IncrementValue + IncrementAmount;
    System.out.println("Wrist Increment Value now: " + IncrementValue);
  }

  public double getIncrementValue() {
    return IncrementValue;
  }

  @Override
  public void periodic() {}

  public void stop() {
    WRIST_MOTOR.set(0);
  }

  public void setupShuffleboard() {
    GenericEntry entry2 = WRIST_TAB.add("Desired DEG", 30).getEntry();
    WRIST_TAB.add(
        "GoTo Desired DEG", new InstantCommand(() -> setDegrees(entry2.get().getDouble())));
    WRIST_TAB.addDouble("Degrees", () -> getDegrees());
    WRIST_TAB.addDouble("Current Amps", () -> WRIST_MOTOR.getStatorCurrent().getValueAsDouble());
    WRIST_TAB.addDouble("Current volts", () -> WRIST_MOTOR.getMotorVoltage().getValueAsDouble());
    WRIST_TAB.addDouble("Error", () -> WRIST_MOTOR.getClosedLoopError().getValueAsDouble());
  }
}
