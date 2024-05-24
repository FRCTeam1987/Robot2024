// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Candles extends SubsystemBase {

  public enum CandleSide {
    BOTH(0),
    LEFT(1),
    RIGHT(2);

    private int candle;

    CandleSide(int i) {
      this.candle = i;
    } 
  
    public int side() {
      return candle;
    }
  }

  private final CANdle LEFT_CANDLE;
  private final CANdle RIGHT_CANDLE;

  /** Creates a new Candles. */
  public Candles(int LEFT_CANDLE_ID, int RIGHT_CANDLE_ID) {
    LEFT_CANDLE = new CANdle(LEFT_CANDLE_ID, "canfd");
    RIGHT_CANDLE = new CANdle(RIGHT_CANDLE_ID, "canfd");
  }

  public void setAnimation(CandleSide side, Animation animation) {
    switch(side) {
      case LEFT:
        LEFT_CANDLE.clearAnimation(0);
        LEFT_CANDLE.animate(animation);
        break;
      case RIGHT:
        RIGHT_CANDLE.clearAnimation(0);
        RIGHT_CANDLE.animate(animation);
        break;
      case BOTH:
        LEFT_CANDLE.clearAnimation(0);
        RIGHT_CANDLE.clearAnimation(0);
        LEFT_CANDLE.animate(animation);
        RIGHT_CANDLE.animate(animation);
        break;
    }
  }

  public void stop() {
    LEFT_CANDLE.clearAnimation(0);
    RIGHT_CANDLE.clearAnimation(0);
    LEFT_CANDLE.clearAnimation(1);
    RIGHT_CANDLE.clearAnimation(1);
  }

  public void setColor(CandleSide side, Color8Bit color) {
    switch(side) {
      case LEFT:
        LEFT_CANDLE.setLEDs(color.red, color.green, color.blue);
        break;
      case RIGHT:
        RIGHT_CANDLE.setLEDs(color.red, color.green, color.blue);
        break;
      case BOTH:
        LEFT_CANDLE.setLEDs(color.red, color.green, color.blue);
        RIGHT_CANDLE.setLEDs(color.red, color.green, color.blue);
        break;
    }
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
