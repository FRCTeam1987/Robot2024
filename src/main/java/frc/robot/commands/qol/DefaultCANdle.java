// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.qol;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Candles;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Util;

public class DefaultCANdle extends Command {
  private final Shooter SHOOTER;
  private final Candles CANDLES;

  /** Creates a new DefaultCANdle. */
  public DefaultCANdle(Candles CANDLES, Shooter SHOOTER) {
    this.CANDLES = CANDLES;
    this.SHOOTER = SHOOTER;

    addRequirements(CANDLES);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!SHOOTER.isRearBroken()) {
      CANDLES.setColor(Candles.CandleSide.LEFT, new Color8Bit(Color.kRed));
      return;
    }

    if (!SHOOTER.isCenterBroken()) {
      CANDLES.setColor(Candles.CandleSide.LEFT, new Color8Bit(Color.kBrown));
      return;
    }

      CANDLES.setColor(Candles.CandleSide.LEFT, new Color8Bit(Color.kAqua));
    if (!Util.isValidShot()) {
      CANDLES.setColor(Candles.CandleSide.RIGHT, new Color8Bit(Color.kRed));
      return;
    }

    // if (Util.isWithinTolerance(Limelight.getTY(SPEAKER_LIMELIGHT), 0.0, 1)
    //     && ((Timer.getFPGATimestamp()) / BLINK_CONSTANT) == 0) {
    //   CANDLES.setColorRightOff();
    //   return;
    // }

    CANDLES.setColor(Candles.CandleSide.RIGHT, new Color8Bit(Color.kGreen));
  }

  @Override
  public void end(boolean interrupted) {}
}
