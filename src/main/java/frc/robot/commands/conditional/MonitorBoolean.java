// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conditional;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public class MonitorBoolean extends Command {
  private BooleanSupplier monitorBoolean;
  private boolean latchSuccess;
  private Runnable onSuccess;
  private Runnable onFail;

  /** Creates a new MonitorLineBreak. */
  public MonitorBoolean(BooleanSupplier monitor, Runnable runOnSuccess, Runnable runOnFail) {
    this.monitorBoolean = monitor;
    this.onSuccess = runOnSuccess;
    this.onFail = runOnFail;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (monitorBoolean.getAsBoolean()) latchSuccess = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (latchSuccess) {
      onSuccess.run();
      return;
    } else {
      onFail.run();
      return;
    }
    // if (interrupted) {
    //   onFail.run();
    // } else {
    //   onSuccess.run();
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return latchSuccess;
  }
}
