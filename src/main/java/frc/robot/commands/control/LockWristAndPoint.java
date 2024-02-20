// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.movement.PointAtAprilTag;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LockWristAndPoint extends ParallelCommandGroup {
  /** Creates a new AimbotNote. */
  public LockWristAndPoint(Shooter shooter, Wrist wrist, Drivetrain drivetrain) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    addCommands(
        new PointAtAprilTag(drivetrain, Constants.LIMELIGHT, Constants.LIMELIGHT_SCORING),
        new AimLockWrist(wrist));
  }
}
