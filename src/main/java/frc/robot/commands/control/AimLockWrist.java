// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.wrist.Wrist;

public class AimLockWrist extends Command {
  private String scoringLimelight;
  private Wrist wrist;

  /** Creates a new AimLockWrist. */
  public AimLockWrist(Wrist wrist, String scoringLimelight) {
    this.wrist = wrist;
    this.scoringLimelight = scoringLimelight;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.get().SHOOTER.isLineBreakBroken()) {
    double degrees = Constants.DISTANCE_WRIST_ANGLE_MAP.get(LimelightHelpers.getTY(scoringLimelight));
    if (degrees < 21 || degrees > 42) {
      DriverStation.reportError("WristAim Map Out of Range", false);
    } else {
      wrist.setDegrees(degrees);
    }
    } else {
      wrist.goHome();
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
