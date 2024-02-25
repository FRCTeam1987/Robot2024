// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNoteSequence extends SequentialCommandGroup {

  private final Debouncer hasNote = new Debouncer(0.00, DebounceType.kRising);

  /** Creates a new IntakeNoteSequence. */
  public IntakeNoteSequence(Shooter shooter, Intake intake, Wrist wrist, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(
            () -> {
              shooter.setFeederVoltage(Constants.FEEDER_FEEDFWD_VOLTS);
              intake.setVolts(Constants.INTAKE_COLLECT_VOLTS);
              wrist.setDegrees(WristConstants.INITIAL_ANGLE_DEGREES);
              elevator.goHome();
            },
            shooter,
            intake,
            wrist),
        new WaitCommand(0.6),
        new WaitUntilCommand(() -> (shooter.getFeederCurrent() > 24 || shooter.isLineBreakBroken())),
        new InstantCommand(
            () -> {
              intake.stopTop();
            },
            intake),
        new WaitUntilCommand(
            () -> hasNote.calculate(shooter.isLineBreakBroken())), // probably debounce this
        new InstantCommand(
            () -> {
              shooter.stopFeeder();
              intake.stopCollecting();
            },
            shooter,
            intake));
        // new InstantCommand(() -> wrist.goHome(), wrist));
  }
}
