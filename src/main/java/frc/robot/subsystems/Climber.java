// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {
  private final TalonFX CLIMB_LEFT;
  private final TalonFX CLIMB_RIGHT;

  /** Creates a new Climber. */
  public Climber(int CLIMB_LEFT_ID, int CLIMB_RIGHT_ID) {
    CLIMB_LEFT = new TalonFX(CLIMB_LEFT_ID, "canfd");
    CLIMB_RIGHT = new TalonFX(CLIMB_RIGHT_ID, "canfd");

    // -0.2 RIGHT OPEN
    // 1.8 CLOSED RIGHT

    TalonFXConfiguration extensionConfig = new TalonFXConfiguration();

    extensionConfig.Slot0.kP = 30.0;
    extensionConfig.Slot0.kI = 0.0;
    extensionConfig.Slot0.kD = 0.0;

    CLIMB_LEFT.getConfigurator().apply(extensionConfig);
    CLIMB_RIGHT.getConfigurator().apply(extensionConfig);
    CLIMB_RIGHT.setInverted(true);
    CLIMB_LEFT.setNeutralMode(NeutralModeValue.Brake);
    CLIMB_RIGHT.setNeutralMode(NeutralModeValue.Brake);
  }

  public void zeroMotors() {
    CLIMB_LEFT.setPosition(0.0);
    CLIMB_RIGHT.setPosition(0.0);
  }

  public void close() {
    CLIMB_LEFT.setControl(new PositionVoltage(2.0));
    CLIMB_RIGHT.setControl(new PositionVoltage(2.0));
  }

  public boolean isClosed() {
    return ((CLIMB_LEFT.getClosedLoopError().getValueAsDouble() < 0.55)
        && (CLIMB_RIGHT.getClosedLoopError().getValueAsDouble() < 0.55));
  }

  public void setSpeeds(double speed) {
    CLIMB_LEFT.setVoltage(speed);
    CLIMB_RIGHT.setVoltage(speed);
  }

  public void stopLeft(boolean direction) {
    CLIMB_LEFT.setVoltage(direction ? Constants.Climber.CLIMBER_MAINTAIN_VOLTAGE : 0);
  }

  public void stopRight(boolean direction) {
    CLIMB_RIGHT.setVoltage(direction ? Constants.Climber.CLIMBER_MAINTAIN_VOLTAGE : 0);
  }

  public void stopAll() {
    CLIMB_LEFT.setVoltage(0);
    CLIMB_RIGHT.setVoltage(0);
  }

  public double getLeftCurrent() {
    return CLIMB_LEFT.getStatorCurrent().getValueAsDouble();
  }

  public double getRightCurrent() {
    return CLIMB_LEFT.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
