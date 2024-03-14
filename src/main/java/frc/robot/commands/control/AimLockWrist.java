// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.util.InterpolatingDouble;

public class AimLockWrist extends Command {
  private final Wrist wrist;
  private final Vision speakerPhoton;
  private final Shooter shooter;

  /** Creates a new AimLockWrist. */
  public AimLockWrist(Wrist wrist, Shooter shooter, Elevator elevator, Vision speakerPhoton) {
    this.shooter = shooter;
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
    if (RobotContainer.isClimbPrimed) {
      return;
    }
    if (RobotContainer.isForwardAmpPrimed || RobotContainer.isReverseAmpPrimed) {
      return;
    }
    if (speakerPhoton.hasTargets()) {
      final double ty = speakerPhoton.getPitchVal();
      SmartDashboard.putNumber("ty", ty);
      if (shooter.isCenterBroken()) {
        double Pitch = speakerPhoton.getPitchVal();
        double degrees =
            Constants.DISTANCE_TO_WRISTANGLE_RELATIVE_SPEAKER.getInterpolated(
                        new InterpolatingDouble(
                            Constants.PITCH_TO_DISTANCE_RELATIVE_SPEAKER.getInterpolated(
                                    new InterpolatingDouble(Pitch))
                                .value))
                    .value;
        wrist.setDegrees(degrees);

      } else {
        wrist.goHome();
      }
    }
  }
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
