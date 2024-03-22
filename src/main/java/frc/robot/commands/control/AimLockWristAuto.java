// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.util.Util;

public class AimLockWristAuto extends Command {
  private final Wrist wrist;
  private final Vision speakerPhoton;

  /** Creates a new AimLockWrist. */
  public AimLockWristAuto(Wrist wrist, Vision speakerPhoton) {
    this.wrist = wrist;
    this.speakerPhoton = speakerPhoton;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (speakerPhoton.hasTargets()) {
      double degrees = Util.getInterpolatedWristAngle(speakerPhoton);
      DriverStation.reportWarning("TRYING DEGREES " + degrees, false);
      wrist.setDegrees(degrees);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
