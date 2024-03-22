// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.qol;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Candles;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.util.Util;

public class DefaultCANdle extends Command {
  private final Shooter SHOOTER;
  private final Candles CANDLES;
  private final Vision PHOTON_SPEAKER;
  private final double BLINK_CONSTANT = 0.002; // in seconds?

  /** Creates a new DefaultCANdle. */
  public DefaultCANdle(Candles CANDLES, Shooter SHOOTER, Vision PHOTON_SPEAKER) {
    this.CANDLES = CANDLES;
    this.SHOOTER = SHOOTER;
    this.PHOTON_SPEAKER = PHOTON_SPEAKER;
    addRequirements(CANDLES);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!SHOOTER.isRearBroken()) {
      CANDLES.setColorLeftRed();
      return;
    }

    if (!SHOOTER.isCenterBroken()) {
      CANDLES.setColorLeftBrown();

      return;
    }

    CANDLES.setColorLeft(0, 128, 128);
    if (!PHOTON_SPEAKER.hasTargets()) {
      CANDLES.setColorRightRed();
      return;
    }

    double dist = Util.getInterpolatedDistance(PHOTON_SPEAKER);

    if (dist < 2.25 && dist > 4.25) {
      CANDLES.setColorRightRed();
      return;
    }

    if (Util.isWithinTolerance(PHOTON_SPEAKER.getYawVal(), 0.0, 1)
        && ((Timer.getFPGATimestamp()) / BLINK_CONSTANT) == 0) {
      CANDLES.setColorRightOff();
      return;
    }

    CANDLES.setColorRightGreen();
  }

  @Override
  public void end(boolean interrupted) {}
}
