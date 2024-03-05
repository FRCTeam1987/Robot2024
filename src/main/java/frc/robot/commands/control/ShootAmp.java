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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

public class ShootAmp extends SequentialCommandGroup {
  /** Creates a new IntakeNoteSequence. */
  private Debouncer lineBreakDebouncer;

  private static final double DEBOUNCE_TIME = 0.06;

  public ShootAmp(Shooter shooter, Elevator elevator, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    lineBreakDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);

    addCommands(
        new InstantCommand(
            () -> {
              shooter.setRPMShoot(500);
              elevator.setLengthInches(6.2);
              // wrist.setDegrees(110.0);  // 25.0 + 90.0
            },
            shooter,
            elevator),
        new WaitCommand(0.5), // reset for isAtSetpoint commands to level out
        new InstantCommand(() -> wrist.setDegrees(110.0), wrist),
        new WaitUntilCommand(() -> (shooter.isShooterAtSetpoint() && wrist.isAtSetpoint())),
        new WaitCommand(0.5), // Time for wrist to get to position
        new InstantCommand(
            () -> shooter.setFeederVoltage(Constants.FEEDER_SHOOT_VOLTS),
            shooter), // Constants.FEEDER_FEEDFWD_VOLTS
        new WaitUntilCommand(
            () ->
                lineBreakDebouncer.calculate(!shooter.isCenterBroken())), // probably debounce this
        new InstantCommand(
            () -> {
              shooter.stopFeeder();
            },
            shooter),
        new WaitUntilCommand(() -> lineBreakDebouncer.calculate(shooter.isCenterBroken())),
        new InstantCommand(() -> wrist.setDegrees(40.0), shooter),
        new WaitCommand(0.2),
        new InstantCommand(
            () -> {
              shooter.stopShooter();
              elevator.goHome();
              wrist.setDegrees(25.0);
            },
            shooter,
            elevator,
            wrist));
  }
}
