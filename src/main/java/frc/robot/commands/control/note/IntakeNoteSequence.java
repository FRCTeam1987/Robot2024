// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.note;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNoteSequence extends SequentialCommandGroup {

  // private final Debouncer hasNote = new Debouncer(0.02, DebounceType.kRising)s

  /** Creates a new IntakeNoteSequence. */
  public IntakeNoteSequence(Shooter shooter, Intake intake, Wrist wrist, Elevator elevator) {
    addCommands(
        new InstantCommand(
            () -> {
              shooter.setFeederVoltage(Constants.Shooter.FEEDER_FEEDFWD_VOLTS);
              // TODO PUT THIS SOMEWHERE ELSE
              shooter.setRPMShoot(Constants.Shooter.SHOOTER_IDLE_RPM_CLOSE);
              intake.setRPM(Constants.INTAKE_RPM);
              wrist.setDegrees(12); // testing
              elevator.goHome();
            },
            shooter,
            intake,
            wrist),
        new WaitCommand(0.1),
        new WaitUntilCommand(shooter::isRearBroken),
        new InstantCommand(() -> RobotContainer.disableTeleopPointToNote()),
        // new InstantCommand(intake::stopTop, intake),
        new WaitUntilCommand(() -> shooter.isCenterBroken()),
        new InstantCommand(
            () -> {
              shooter.stopFeeder();
              intake.stopCollecting();
            },
            shooter,
            intake));
  }

  public IntakeNoteSequence(
      Shooter shooter, Intake intake, Elevator elevator, boolean val, double IntakeVoltage) {
    addCommands(
        new InstantCommand(
            () -> {
              shooter.setFeederVoltage(Constants.Shooter.FEEDER_FEEDFWD_VOLTS_AGRESSIVE);
              intake.setRPM(Constants.INTAKE_RPM);
              elevator.goHome();
            },
            shooter,
            intake),
        new WaitCommand(0.1),
        new WaitUntilCommand(shooter::isRearBroken),
        // new InstantCommand(intake::stopTop, intake),
        new ConditionalCommand(
            new WaitUntilCommand(() -> shooter.isCenterBroken()), new InstantCommand(), () -> val),
        new WaitUntilCommand(() -> shooter.isCenterBroken()),
        new InstantCommand(
            () -> {
              shooter.stopFeeder();
              intake.stopCollecting();
            },
            shooter,
            intake));
  }
}
