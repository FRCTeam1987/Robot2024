package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.zeroing.ZeroWrist;
import frc.robot.constants.Constants;

public class Wrist extends SubsystemBase {
  public static double incrementAimbot = 1.0;
  private final TalonFX WRIST_MOTOR;
  private final ShuffleboardTab WRIST_TAB = Shuffleboard.getTab("WRIST");
  private double IncrementValue = 0.0;

  // Constructor
  public Wrist(final int wristMotorID) {
    WRIST_MOTOR = new TalonFX(wristMotorID, "rio");
    final TalonFXConfiguration WRIST_CONFIG = new TalonFXConfiguration();

    WRIST_CONFIG.Slot0.kP = 250.0; // Constants.Wrist.WRIST_KP; 500
    WRIST_CONFIG.Slot0.kI = Constants.Wrist.WRIST_KI;
    WRIST_CONFIG.Slot0.kD = Constants.Wrist.WRIST_KD;
    WRIST_CONFIG.Slot0.kV = Constants.Wrist.WRIST_KV;

    WRIST_CONFIG.CurrentLimits.StatorCurrentLimit = Constants.Wrist.WRIST_CURRENT_LIMIT;
    // WRIST_CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.Wrist.WRIST_CURRENT_LIMIT;
    // WRIST_CONFIG.SupplyCurrentT
    // WRIST_CONFIG.CurrentLimits.SupplyCurrentLimitEnable  = true;
    WRIST_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

    WRIST_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 15; // 50
    // Constants.Wrist.WRIST_MOTION_CRUISE_VELOCITY;
    WRIST_CONFIG.MotionMagic.MotionMagicAcceleration =
        WRIST_CONFIG.MotionMagic.MotionMagicCruiseVelocity / 4.0;
    // Constants.Wrist.WRIST_MOTION_ACCELERATION;
    // WRIST_CONFIG.MotionMagic.MotionMagicJerk = 10;
    WRIST_CONFIG.Feedback.SensorToMechanismRatio = (120.0 / 10.0) * (80.0 / 12.0);
    // (100.0 / 10.0) * (36.0 / 18.0) * (3.0 / 1.0); // 66:1
    // Constants.Wrist.WRIST_MOTION_JERK;

    WRIST_MOTOR.getConfigurator().apply(WRIST_CONFIG);
    setZero();
    WRIST_MOTOR.setNeutralMode(NeutralModeValue.Brake);

    setupShuffleboard();
  }

  public void goHome() {
    setDegrees(Constants.Wrist.INITIAL_ANGLE_DEGREES);
  }

  public void setCoast() {
    WRIST_MOTOR.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrake() {
    WRIST_MOTOR.setNeutralMode(NeutralModeValue.Brake);
  }

  public void zeroSensor() {
    WRIST_MOTOR.setPosition(Constants.Wrist.INITIAL_ANGLE_DEGREES / 360.0);
  }

  public double getError() {
    return WRIST_MOTOR.getClosedLoopError().getValueAsDouble();
  }

  public boolean isAtSetpoint() {
    return WRIST_MOTOR.getClosedLoopError().getValueAsDouble()
        < Constants.Wrist.WRIST_ALLOWABLE_ERROR;
  }

  public double getVelocity() {
    return WRIST_MOTOR.getVelocity().getValueAsDouble();
  }

  public void setVoltage(double volts) {
    WRIST_MOTOR.setVoltage(volts);
  }

  public double getDegrees() {
    // initial angle irl is 13 degrees
    // 44 degrees at 1.67 rotations
    // Reduction is 18T to 36T pulley, 10T to 100T gear
    return WRIST_MOTOR.getPosition().getValueAsDouble() * 360.0;
    // * Constants.Wrist.CONVERSION_FACTOR_ROTS_TO_DEGREES;
  }

  public void setDegrees(double degrees) {
    degrees = degrees + IncrementValue;

    if (degrees > Constants.Wrist.WRIST_MAX_DEG || degrees < Constants.Wrist.WRIST_MIN_DEG) {
      System.out.println("Out of Wrist Range! " + degrees);
    } else {
      double arbFF = 0.4 * Math.sin(Math.toRadians(90.0 - degrees));
      // DriverStation.reportWarning("Wrist degrees: " + degrees, false);
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

  public void stop() {
    WRIST_MOTOR.set(0);
  }

  public void coast() {
    WRIST_MOTOR.setNeutralMode(NeutralModeValue.Coast);
  }

  public void brake() {
    WRIST_MOTOR.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setZero() {
    WRIST_MOTOR.setPosition(Constants.Wrist.INITIAL_ANGLE_DEGREES / 360.0);
  }

  public void setupShuffleboard() {
    GenericEntry entry2 = WRIST_TAB.add("Desired DEG", 28).getEntry();
    WRIST_TAB.add("+1 AIMBOT DEG", new InstantCommand(() -> Wrist.incrementAimbot++));
    WRIST_TAB.add("-1 AIMBOT DEG", new InstantCommand(() -> Wrist.incrementAimbot--));
    WRIST_TAB.add("ZERO WRIST", new ZeroWrist(this));
    WRIST_TAB.add(
        "GoTo Desired DEG", new InstantCommand(() -> setDegrees(entry2.get().getDouble())));
    WRIST_TAB.add("Set Coast", new InstantCommand(this::setCoast, this).ignoringDisable(true));
    WRIST_TAB.add("Set Brake", new InstantCommand(this::setBrake, this).ignoringDisable(true));
    WRIST_TAB.addDouble("Degrees", this::getDegrees);
    if (Constants.shouldShuffleboard) {
      WRIST_TAB.addDouble("Wrist degrees with offset", this::getDegrees);
      WRIST_TAB.addDouble("Current Amps", () -> WRIST_MOTOR.getStatorCurrent().getValueAsDouble());
      WRIST_TAB.addDouble("Current volts", () -> WRIST_MOTOR.getMotorVoltage().getValueAsDouble());
      WRIST_TAB.addDouble("Error", () -> WRIST_MOTOR.getClosedLoopError().getValueAsDouble());
    }
  }
}
