// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.qol;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Util;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Candles;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.util.InterpolatingDouble;

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
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(Timer.getFPGATimestamp());
    // testing
    if (SHOOTER.isRearBroken()) {
      if (SHOOTER.isCenterBroken()) {
        if (PHOTON_SPEAKER.hasTargets()) {
          double dist =
              Constants.PITCH_TO_DISTANCE_RELATIVE_SPEAKER.getInterpolated(
                      new InterpolatingDouble(PHOTON_SPEAKER.getPitchVal()))
                  .value;
          if (dist > 2.25 && dist < 4.25) {
            if (Util.isWithinTolerance(PHOTON_SPEAKER.getYawVal(), 0.0, 1)) {
              if (((Timer.getFPGATimestamp()) / BLINK_CONSTANT) == 0) {
                CANDLES.setColorRight(0, 0, 0);
                return;
              } else {
                CANDLES.setColorRight(0, 255, 0);
                return;
              }
            } else {
              CANDLES.setColorRight(0, 255, 0);
            }
            CANDLES.setColorRight(0, 255, 0);
          } else {
            CANDLES.setColorRight(255, 0, 0);
          }
        } else {
          CANDLES.setColorRight(255, 0, 0);
        }

        CANDLES.setColorLeft(0, 128, 128);
        return;
      }
      CANDLES.setColorLeft(128, 64, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
