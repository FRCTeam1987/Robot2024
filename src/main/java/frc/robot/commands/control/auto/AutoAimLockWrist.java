// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.util.Util;

public class AutoAimLockWrist extends Command {
  private final Wrist wrist;

  /** Creates a new AimLockWrist. */
  public AutoAimLockWrist(Wrist wrist) {
    this.wrist = wrist;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double degrees = Util.getInterpolatedWristAngle();
    // TODO find actual values, prevent wrist collision when the elevator is all the way down.
    DriverStation.reportWarning("Wrist Degrees Angle " + degrees, false);
    if (degrees > 10.0 && degrees < 35.0) {

      return;
    }
    wrist.setDegrees(degrees);
  }

  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("Ended! " + interrupted, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
