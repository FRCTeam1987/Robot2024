// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.note;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class ShootNote extends SequentialCommandGroup {
  private static final double DEBOUNCE_TIME = 0.08;

  /** Creates a new IntakeNoteSequence. */
  private final Debouncer lineBreakDebouncer;

  private final Debouncer shootNoteDebouncer;

  public ShootNote(Shooter shooter, Elevator elevator, double shootRPM) {
    addRequirements(shooter);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    lineBreakDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);
    shootNoteDebouncer = new Debouncer(0.2, DebounceType.kRising);
    // elevator.goHome();
    addCommands(
        new InstantCommand(() -> shooter.setRPMShoot(shootRPM), shooter),
        new WaitCommand(0.1),
        new WaitUntilCommand(() -> shooter.isShooterAtSetpoint()),
        new InstantCommand(
            () -> shooter.setFeederVoltage(Constants.Shooter.FEEDER_SHOOT_VOLTS),
            shooter), // Constants.FEEDER_FEEDFWD_VOLTS
        new WaitUntilCommand(
            () ->
                lineBreakDebouncer.calculate(!shooter.isCenterBroken())), // probably debounce this
        new InstantCommand(shooter::stopFeeder, shooter),
        new WaitUntilCommand(() -> lineBreakDebouncer.calculate(shooter.isCenterBroken())),
        new WaitCommand(0.1),
        new InstantCommand(shooter::stopShooter, shooter));
  }
}
