package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private final TalonFX WRIST_MOTOR;
  private final int allowableError = 100;
  private final ShuffleboardTab WRIST_TAB = Shuffleboard.getTab("WRIST");

  // Constructor
  public Wrist(int wristMotorID) {

    Slot0Configs wristConfig = new Slot0Configs();
    wristConfig.kP = 0.12;
    wristConfig.kI = 0;
    wristConfig.kD = 0;
    wristConfig.kV = 0.01;

    WRIST_MOTOR = new TalonFX(wristMotorID);
    WRIST_MOTOR.getConfigurator().apply(wristConfig);
  }

  public void moveToPositionTicks(double ticks) {
    WRIST_MOTOR.setControl(new PositionVoltage(ticks));
  }

  @Override
  public void periodic() {}

  public void stop() {
    WRIST_MOTOR.set(0);
  }

  public void setupShuffleboard() {
    GenericEntry entry = WRIST_TAB.add("PositionTicks", 400).getEntry();
    WRIST_TAB.add(
        "SetWrist", new InstantCommand(() -> moveToPositionTicks(entry.get().getDouble())));
  }
}
