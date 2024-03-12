// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.qol;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.candle.Candles;

public class AsyncCANdle extends Command {
  private final Candles candles;
  private final Color8Bit color;
  private final long durationMs;
  private long finishtime;

  /** Creates a new Rumble. */
  public AsyncCANdle(Candles candles, Color8Bit color, long durationMs) {
    this.candles = candles;
    this.color = color;
    this.durationMs = durationMs;
    addRequirements(candles);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    long time = System.currentTimeMillis();
    this.finishtime = time + durationMs;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    candles.setColor(color.red, color.green, color.blue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() >= finishtime;
  }
}
