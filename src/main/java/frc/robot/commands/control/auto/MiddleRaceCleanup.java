// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.control.ShootSubwooferFirstHalf;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleRaceCleanup extends SequentialCommandGroup {
  /** Creates a new MiddleRaceCleanup. */
  public MiddleRaceCleanup(
      final CommandSwerveDrivetrain drivetrain,
      final Intake intake,
      final Elevator elevator,
      final Wrist wrist,
      final Shooter shooter,
      final Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Temporarily hard code flipping the path.
    final PathPlannerPath step1 = PathPlannerPath.fromPathFile("Middle Race 1").flipPath();
    final PathPlannerPath step2Success =
        PathPlannerPath.fromPathFile("Middle Race 2 Success").flipPath();
    final PathPlannerPath step2Fail = PathPlannerPath.fromPathFile("Middle Race 2 Fail").flipPath();
    final PathPlannerPath step3 = PathPlannerPath.fromPathFile("Middle Race 3").flipPath();
    addCommands(
        new ShootSubwooferFirstHalf(elevator, wrist, shooter),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> {
                      drivetrain.seedFieldRelative(step1.getPreviewStartingHolonomicPose());
                    }),
                AutoBuilder.followPath(step1),
                new ConditionalCommand(
                    AutoBuilder.followPath(step2Success),
                    AutoBuilder.followPath(step2Fail),
                    () -> shooter.isRearBroken() || shooter.isCenterBroken()),
                new AutoCollectNote(step2Success.getGoalEndState().getVelocity()),
                new AutoAimAndShoot(drivetrain, shooter),
                AutoBuilder.followPath(step3),
                new AutoCollectNote(step3.getGoalEndState().getVelocity()),
                new AutoAimAndShoot(drivetrain, shooter)),
            new AutoAimLockWrist(wrist),
            new AutoIdleShooter(shooter)));
  }
}
