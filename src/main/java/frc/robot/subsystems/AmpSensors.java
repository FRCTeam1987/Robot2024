// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class AmpSensors extends SubsystemBase {

  private DigitalInput m_sensor_left;
  private DigitalInput m_sensor_right;

  /** Creates a new AmpSensors. */
  public AmpSensors() {
    m_sensor_left = new DigitalInput(Constants.Wrist.PROXIMITY_SENSOR_LEFT_ID);
    m_sensor_right = new DigitalInput(Constants.Wrist.PROXIMITY_SENSOR_RIGHT_ID);
    Shuffleboard.getTab("MAIN").addBoolean("Amp Sensor", () -> getBothSensors());
  }

  public boolean getSensorLeft() {
    return m_sensor_left.get();
  }

  public boolean getSensorRight() {
    return m_sensor_right.get();
  }

  public boolean getBothSensors() {
    return this.getBothSensors(true);
  }

  public boolean getBothSensors(boolean shouldReturnAsAND) {
    if (shouldReturnAsAND) {
      return this.getSensorLeft() && this.getSensorRight();
    } else {
      return this.getSensorLeft() || this.getSensorRight();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
