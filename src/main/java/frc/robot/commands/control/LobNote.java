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
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LobNote extends SequentialCommandGroup {
  private final Debouncer lineBreakDebouncer;

  private final double DEBOUNCE_TIME = 0.08;

  // Add your commands in the addCommands() call, e.g.
  // addCommands(new FooCommand(), new BarCommand());

  /** Creates a new LobNote. */
  public LobNote(Shooter SHOOTER, Wrist WRIST, Elevator ELEVATOR) {

    lineBreakDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(SHOOTER, WRIST, ELEVATOR);
    addCommands(
        new InstantCommand(
            () -> {
              WRIST.setDegrees(33);
              SHOOTER.setRPMShoot(ShooterConstants.SHOOTER_LOB_RPM);
              ELEVATOR.setLengthInches(0);
            },
            SHOOTER,
            WRIST),
        new WaitCommand(0.1), // reset for isAtSetpoint commands to level out
        new WaitUntilCommand(
            () -> WRIST.isAtSetpoint() && SHOOTER.isShooterAtSetpoint() && ELEVATOR.isAtSetpoint()),
        new WaitCommand(0.4), // Time for writst to get to position
        new InstantCommand(
            () -> SHOOTER.setFeederVoltage(ShooterConstants.FEEDER_FEEDFWD_VOLTS_AGRESSIVE),
            SHOOTER), // Constants.FEEDER_FEEDFWD_VOLTS
        new WaitUntilCommand(
            () ->
                lineBreakDebouncer.calculate(!SHOOTER.isCenterBroken())), // probably debounce this
        new InstantCommand(SHOOTER::stopFeeder, SHOOTER),
        new WaitUntilCommand(() -> lineBreakDebouncer.calculate(SHOOTER.isCenterBroken())),
        new InstantCommand(
            () -> {
              SHOOTER.stopShooter();
              ELEVATOR.goHome();
            },
            SHOOTER),
        new WaitCommand(0.1));
  }
}
