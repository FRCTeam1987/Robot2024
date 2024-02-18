// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNoteSequence extends SequentialCommandGroup {
  /** Creates a new IntakeNoteSequence. */
  public IntakeNoteSequence(Shooter shooter, Intake intake, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(
            () -> {
              shooter.setFeederVoltage(Constants.FEEDER_FEEDFWD_VOLTS);
              intake.setVolts(Constants.INTAKE_COLLECT_VOLTS);
              // wrist.moveToPositionDegrees(40);
            },
            shooter,
            intake,
            wrist),
        new WaitUntilCommand(() -> shooter.isLineBreakBroken()), // probably debounce this
        new InstantCommand(
            () -> {
              shooter.setFeederVoltage(Constants.FEEDER_RETRACT_VOLTS);
              intake.stopCollecting();
            },
            shooter,
            intake),
        new WaitUntilCommand(() -> !shooter.isLineBreakBroken()),
        new InstantCommand(() -> shooter.stopFeeder(), shooter),
        new InstantCommand(() -> wrist.moveToPositionRotations(0), wrist));
  }
}
