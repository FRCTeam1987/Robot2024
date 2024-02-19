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
import frc.robot.Util;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.Shooter;

public class ShootNote extends SequentialCommandGroup {
  /** Creates a new IntakeNoteSequence. */
  private Debouncer lineBreakDebouncer;

  private static final double DEBOUNCE_TIME = 0.06;

  public ShootNote(Shooter shooter, double shootRPM) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    lineBreakDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);

    addCommands(
        new InstantCommand(
            () -> {
              shooter.setRPMShoot(shootRPM);
            },
            shooter),
        new WaitCommand(0.1), // reset for isAtSetpoint commands to level out
        new WaitUntilCommand(() -> shooter.isShooterAtSetpoint()),
        new WaitCommand(0.5), // Time for writst to get to position
        new InstantCommand(
            () -> shooter.setFeederVoltage(Constants.FEEDER_FEEDFWD_VOLTS),
            shooter), // Constants.FEEDER_FEEDFWD_VOLTS
        new WaitUntilCommand(
            () ->
                lineBreakDebouncer.calculate(
                    !shooter.isLineBreakBroken())), // probably debounce this
        new InstantCommand(
            () -> {
              shooter.stopFeeder();
            },
            shooter),
        new WaitUntilCommand(() -> lineBreakDebouncer.calculate(shooter.isLineBreakBroken())),
        new InstantCommand(
            () -> {
              shooter.stopShooter();
            },
            shooter));
  }
}
