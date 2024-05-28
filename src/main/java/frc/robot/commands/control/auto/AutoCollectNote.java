// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.control.note.IntakeNoteSequenceAuto;
import frc.robot.commands.movement.DriveToNote2;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCollectNote extends ParallelDeadlineGroup {
  /** Creates a new AutoCollectNote. */
  public AutoCollectNote(
      final CommandSwerveDrivetrain drivetrain,
      final Vision vision,
      final double initialVelocity,
      final Shooter shooter,
      final Intake intake,
      final Elevator elevator) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new IntakeNoteSequenceAuto(shooter, intake, elevator));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> RobotContainer.WRIST.enableWristLockdown())
            .andThen(new DriveToNote2(drivetrain, vision, initialVelocity))
            .finallyDo(() -> RobotContainer.WRIST.disableWristLockdown()));
    // addCommands(new DriveToNote2(drivetrain, vision, initialVelocity));
  }
}
