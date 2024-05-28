// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.movement.PointAtSpeaker;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Util;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAimAndShoot extends SequentialCommandGroup {
  /** Creates a new AutoAimAndShoot. */
  public AutoAimAndShoot(final CommandSwerveDrivetrain drivetrain, final Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelDeadlineGroup(
            new WaitUntilCommand(() -> Util.isPointedAtSpeaker(drivetrain) && shooter.isShooterAtSetpoint()).withTimeout(0.5),
            new PointAtSpeaker(drivetrain, () -> 0.0, () -> 0.0, () -> 0.0)),
        new InstantShoot(shooter));
  }
}
