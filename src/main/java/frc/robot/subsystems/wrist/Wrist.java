package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants;

public class Wrist extends SubsystemBase {
  private final TalonFX WRIST_MOTOR;
  private final int allowableError = 100;
  private final ShuffleboardTab WRIST_TAB = Shuffleboard.getTab("WRIST");

  // Constructor
  public Wrist(int wristMotorID) {


    TalonFXConfiguration cfg = new TalonFXConfiguration();

    Slot0Configs wristConfig = new Slot0Configs();
    wristConfig.kP = 1.3;
    wristConfig.kI = 0;
    wristConfig.kD = 0;
    wristConfig.kV = 0.01;

    cfg.Slot0 = wristConfig;
    cfg.CurrentLimits.StatorCurrentLimit = 15;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.Feedback.FeedbackRotorOffset = Constants.WRIST_OFFSET;

    WRIST_MOTOR = new TalonFX(wristMotorID, "rio");
    WRIST_MOTOR.getConfigurator().apply(cfg);
    setupShuffleboard();
  }

  public void moveToPositionRotations(double rots) {
    WRIST_MOTOR.setControl(new PositionVoltage(rots));
  }

  @Override
  public void periodic() {}

  public void stop() {
    WRIST_MOTOR.set(0);
  }

  public void setupShuffleboard() {
    GenericEntry entry = WRIST_TAB.add("PositionRots", 400).getEntry();
    WRIST_TAB.add(
        "SetWrist", new InstantCommand(() -> moveToPositionRotations(entry.get().getDouble())));
    WRIST_TAB.addDouble("Ticks", () -> WRIST_MOTOR.getRotorPosition().getValueAsDouble());
  }
}
