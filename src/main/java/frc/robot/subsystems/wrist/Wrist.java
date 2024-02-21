package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
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

  // Constructor
  public Wrist(final int wristMotorID) {
    WRIST_MOTOR = new TalonFX(wristMotorID, "rio");
    final TalonFXConfiguration WRIST_CONFIG = new TalonFXConfiguration();

    WRIST_CONFIG.Slot0.kP = WristConstants.WRIST_KP;
    WRIST_CONFIG.Slot0.kI = WristConstants.WRIST_KI;
    WRIST_CONFIG.Slot0.kD = WristConstants.WRIST_KD;
    WRIST_CONFIG.Slot0.kV = WristConstants.WRIST_KV;

    WRIST_CONFIG.CurrentLimits.StatorCurrentLimit = WristConstants.WRIST_CURRENT_LIMIT;
    WRIST_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

    WRIST_CONFIG.MotionMagic.MotionMagicAcceleration = WristConstants.WRIST_MOTION_ACCELERATION;
    WRIST_CONFIG.MotionMagic.MotionMagicCruiseVelocity =
        WristConstants.WRIST_MOTION_CRUISE_VELOCITY;
    // WRIST_CONFIG.MotionMagic.MotionMagicJerk = WristConstants.WRIST_MOTION_JERK;

    WRIST_MOTOR.getConfigurator().apply(WRIST_CONFIG);
    WRIST_MOTOR.setPosition(0);
    WRIST_MOTOR.setNeutralMode(NeutralModeValue.Brake);

    setupShuffleboard();
  }

  public void goHome() {
    WRIST_MOTOR.setControl(new PositionVoltage(0));
  }

  public void zeroSensor() {
    WRIST_MOTOR.setPosition(0);
  }

  public boolean isAtSetpoint() {
    return WRIST_MOTOR.getClosedLoopError().getValueAsDouble()
        < WristConstants.WRIST_ALLOWABLE_ERROR;
  }

  public double getDegrees() {
    return WRIST_MOTOR.getPosition().getValueAsDouble()
        * WristConstants.CONVERSION_FACTOR_ROTS_TO_DEGREES;
  }

  public void setDegrees(double degrees) {
    if (degrees > WristConstants.WRIST_MAX_DEG || degrees < WristConstants.WRIST_MIN_DEG) {
      System.out.println("Out of Wrist Range!");
      return;
    } else {
      double arbFF = WristConstants.WRIST_KV * Math.sin(Math.toRadians(90.0 - getDegrees()));
      WRIST_MOTOR.setControl(
          new MotionMagicVoltage(
              (WristConstants.CONVERSION_FACTOR_DEGREES_TO_ROTS * degrees) - 1,
              true,
              arbFF,
              0,
              false,
              false,
              false));
    }
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
