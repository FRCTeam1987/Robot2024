// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
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

  public void setAnimationBoth(Animation animation) {
    LEFT_CANDLE.clearAnimation(0);
    RIGHT_CANDLE.clearAnimation(0);
    LEFT_CANDLE.animate(animation);
    RIGHT_CANDLE.animate(animation);
  }

  public void setAnimationRight(Animation animation) {
    RIGHT_CANDLE.animate(animation);
  }

  public void setAnimationLeft(Animation animation) {
    LEFT_CANDLE.animate(animation);
  }

  public void setColorBoth(int r, int g, int b) {
    LEFT_CANDLE.setLEDs(r, g, b);
    RIGHT_CANDLE.setLEDs(r, g, b);
  }

  public void setColorBothRed() {
    setColorBoth(255, 0, 0);
  }

  public void setColorBothGreen() {
    setColorBoth(0, 255, 0);
  }

  public void setColorBothBlue() {
    setColorBoth(255, 0, 255);
  }

  public void setColorBothBrown() {
    setColorBoth(128, 64, 0);
  }

  public void stop() {
    LEFT_CANDLE.clearAnimation(0);
    RIGHT_CANDLE.clearAnimation(0);
    LEFT_CANDLE.clearAnimation(1);
    RIGHT_CANDLE.clearAnimation(1);
  }

  public void setColorLeft(int r, int g, int b) {
    LEFT_CANDLE.setLEDs(r, g, b);
  }

  public void setColorLeftRed() {
    setColorLeft(255, 0, 0);
  }

  public void setColorLeftGreen() {
    setColorLeft(0, 255, 0);
  }

  public void setColorLeftBlue() {
    setColorLeft(255, 0, 255);
  }

  public void setColorLeftBrown() {
    setColorLeft(128, 64, 0);
  }

  public void setColorLeftOff() {
    setColorLeft(0, 0, 0);
  }

  public void setColorRight(int r, int g, int b) {
    RIGHT_CANDLE.setLEDs(r, g, b);
  }

  public void setColorRightRed() {
    setColorRight(255, 0, 0);
  }

  public void setColorRightGreen() {
    setColorRight(0, 255, 0);
  }

  public void setColorRightBlue() {
    setColorRight(255, 0, 255);
  }

  public void setColorRightBrown() {
    setColorRight(128, 64, 0);
  }

  public void setColorRightOff() {
    setColorRight(0, 0, 0);
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
