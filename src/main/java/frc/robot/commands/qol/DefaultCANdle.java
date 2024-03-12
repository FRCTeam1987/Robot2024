// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.qol;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.candle.Candles;
import frc.robot.subsystems.shooter.Shooter;

public class DefaultCANdle extends Command {
  private Shooter SHOOTER;
  private Candles CANDLES;

  /** Creates a new DefaultCANdle. */
  public DefaultCANdle(Candles CANDLES, Shooter SHOOTER) {
    this.CANDLES = CANDLES;
    this.SHOOTER = SHOOTER;
    addRequirements(CANDLES);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SHOOTER.isRearBroken()) {
      if (SHOOTER.isCenterBroken()) {
        CANDLES.setColor(0, 128, 128);
        return;
      }
      CANDLES.setColor(128, 64, 0);
      return;
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
