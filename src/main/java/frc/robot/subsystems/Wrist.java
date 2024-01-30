package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private final TalonFX wristMotor;
  private final int allowableError = 100;

  // Constructor
  public Wrist(int wristMotorID) {
    wristMotor = new TalonFX(wristMotorID);
  }

  public void moveToPosition(int position) {
    wristMotor.setPosition(position);
  }

  @Override
  public void periodic() {}

  public void stop() {
    wristMotor.set(0);
  }

  public boolean isAtTargetPosition(int targetPosition) {
    int currentPosition =
        (int) Math.round(wristMotor.getPosition().getValue()); // Is getPosition actually in ticks?
    return Math.abs(currentPosition - targetPosition) <= allowableError;
  }
}
