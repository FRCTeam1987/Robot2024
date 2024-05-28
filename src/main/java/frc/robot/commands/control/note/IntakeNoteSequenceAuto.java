// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.note;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.control.auto.AutoState;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNoteSequenceAuto extends SequentialCommandGroup {

  // private final Debouncer hasNote = new Debouncer(0.02, DebounceType.kRising);

  /** Creates a new IntakeNoteSequence. */
  public IntakeNoteSequenceAuto(Shooter shooter, Intake intake, Elevator elevator) {
    addCommands(
        new InstantCommand(
            () -> {
              RobotContainer.setAutoState(AutoState.COLLECTING);
              intake.setRPM(Constants.INTAKE_RPM);
              elevator.goHome();
            },
            intake),
        // new WaitCommand(0.1),  should be able to be removed
        // new WaitUntilCommand(shooter::isRearBroken),
        // new InstantCommand(intake::stopTop, intake),
        new WaitUntilCommand(() -> shooter.isCenterBroken()),
        new InstantCommand(
            () -> {
              shooter.stopFeeder();
              RobotContainer.setAutoState(AutoState.DEFAULT);
              intake.stopCollecting();
            },
            intake));
  }
}
