// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.util.Util;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Madtown extends ParallelCommandGroup {

  public static boolean SHOULD_WATCH_FOR_NOTE = false;
  private static boolean INITIAL_WAS_INTERRUPTED = false;

  /** Creates a new Madtown. */
  public Madtown() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            new ParallelRaceGroup(
                AutoBuilder.buildAuto("Madtown Initial"),
                new WaitUntilCommand(
                    () -> { 
                      final boolean shouldInterrupt = SHOULD_WATCH_FOR_NOTE && RobotContainer.INTAKE_PHOTON.hasTargets() && Math.abs(RobotContainer.INTAKE_PHOTON.getYawVal()) < 10.0;
                      INITIAL_WAS_INTERRUPTED = shouldInterrupt;
                      return shouldInterrupt;
                    })),
            new ConditionalCommand(
                new AutoCollectNote(2.75)  // TODO if this fails near note 4, then try for note 5, probably won't have time in 15 seconds
                    .withTimeout(1.0)
                    .andThen(Util.PathFindToAutoSourceCloseShot())
                    .andThen(
                        new AutoAimAndShoot(RobotContainer.DRIVETRAIN, RobotContainer.SHOOTER)),
                new InstantCommand(),
                () -> INITIAL_WAS_INTERRUPTED),
            new RotateUntilNote(true),
            new AutoCollectNote(2.5),
            Util.PathFindToAutoMadtownShot(),
            new AutoAimAndShoot(RobotContainer.DRIVETRAIN, RobotContainer.SHOOTER),
            new RotateUntilNote(false),
            new AutoCollectNote(2.75),
            Util.PathFindToAutoMadtownShot(),
            new AutoAimAndShoot(RobotContainer.DRIVETRAIN, RobotContainer.SHOOTER)),
        new AutoAimLockWrist(RobotContainer.WRIST, RobotContainer::getAutoState),
        new AutoIdleShooter(RobotContainer.SHOOTER, RobotContainer::getAutoState));
  }
}
