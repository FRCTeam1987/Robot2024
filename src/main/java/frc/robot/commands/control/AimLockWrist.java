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
import frc.robot.util.InterpolatingDouble;

public class AimLockWrist extends Command {
  private Wrist wrist;

  /** Creates a new AimLockWrist. */
  public AimLockWrist(Wrist wrist) {
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
    if (LimelightHelpers.getTV(Constants.LIMELIGHT_SCORING)) {
      final double ty = LimelightHelpers.getTY(Constants.LIMELIGHT_SCORING);
      if (RobotContainer.get().SHOOTER.isLineBreakBroken() && (ty > 2 || ty < 4)) {
        // try {
        double distance =
            LimelightHelpers.calculateDistanceToTarget(
                LimelightHelpers.getTY(Constants.LIMELIGHT_SCORING),
                Constants.SHOOTER_LIMELIGHT_HEIGHT,
                Constants.SPEAKER_APRILTAG_HEIGHT,
                Constants.SHOOTER_LIMELIGHT_ANGLE);
        System.out.println("Calculating for: " + distance);
        double degrees =
            Constants.DISTANCE_WRIST_ANGLE_MAP.getInterpolated(new InterpolatingDouble(distance))
                .value;
        // if (degrees < 21 || degrees > 42) {
        //   DriverStation.reportError("WristAim Map Out of Range", false);
        // } else {
        System.out.println("Degrees attempted: " + degrees);
        wrist.setDegrees(degrees);
        // }
        // } catch (Exception ignored) {
        // }

      } else {
        wrist.goHome();
      }
    } else {
      DriverStation.reportError("Cannot see apriltag", false);
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
