// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    // CLIMB_RIGHT.setInverted(true);

    TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
    // extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // extensionConfig.CurrentLimits.StatorCurrentLimit = ClimberConstants.EXTENSION_CURRENT_LIMIT;

    // CLIMB_LEFT.getConfigurator().apply(extensionConfig);
    // CLIMB_RIGHT.getConfigurator().apply(extensionConfig);
    CLIMB_RIGHT.setInverted(true);
    CLIMB_LEFT.setNeutralMode(NeutralModeValue.Brake);
    CLIMB_RIGHT.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setSpeeds(double speed) {
    CLIMB_LEFT.setVoltage(speed);
    CLIMB_RIGHT.setVoltage(speed);
  }

  public void stopLeft(boolean direction) {
    CLIMB_LEFT.setVoltage(direction ? Constants.CLIMBER_MAINTAIN_VOLTAGE : 0);
  }

  public void stopRight(boolean direction) {
    CLIMB_RIGHT.setVoltage(direction ? Constants.CLIMBER_MAINTAIN_VOLTAGE : 0);
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
