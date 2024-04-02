// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpSensors extends SubsystemBase {

  private DigitalInput m_sensor;

  /** Creates a new AmpSensors. */
  public AmpSensors() {
    m_sensor = new DigitalInput(9);
    Shuffleboard.getTab("MAIN").addBoolean("Amp Sensor", m_sensor::get);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
