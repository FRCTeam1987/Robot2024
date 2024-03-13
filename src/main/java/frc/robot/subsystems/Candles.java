// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candles extends SubsystemBase {
  private final CANdle LEFT_CANDLE;
  private final CANdle RIGHT_CANDLE;

  /** Creates a new Candles. */
  public Candles(int LEFT_CANDLE_ID, int RIGHT_CANDLE_ID) {
    LEFT_CANDLE = new CANdle(LEFT_CANDLE_ID, "canfd");
    RIGHT_CANDLE = new CANdle(RIGHT_CANDLE_ID, "canfd");
  }

  public void setColorBoth(int r, int g, int b) {
    LEFT_CANDLE.setLEDs(r, g, b);
    RIGHT_CANDLE.setLEDs(r, g, b);
  }

  public void setColorLeft(int r, int g, int b) {
    LEFT_CANDLE.setLEDs(r, g, b);
  }

  public void setColorRight(int r, int g, int b) {
    RIGHT_CANDLE.setLEDs(r, g, b);
  }

  public void off() {
    LEFT_CANDLE.setLEDs(0, 0, 0);
    RIGHT_CANDLE.setLEDs(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
