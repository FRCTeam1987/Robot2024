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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class ShootAmp extends SequentialCommandGroup {
  /** Creates a new IntakeNoteSequence. */
  private final Debouncer lineBreakDebouncer;

  private static final double DEBOUNCE_TIME = 0.06;

  public ShootAmp(Shooter shooter, Elevator elevator, Wrist wrist) {
    addRequirements(shooter);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    lineBreakDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);

    addCommands(
        new InstantCommand(
            () -> {
              shooter.setRPMShoot(Constants.Shooter.SHOOTER_AMP_RPM);
              elevator.setLengthInches(Constants.FWD_ELEVATOR_AMP_HEIGHT);
              // wrist.setDegrees(110.0);  // 25.0 + 90.0
            },
            shooter,
            elevator),
        // new WaitUntilCommand(() -> (elevator.isAtSetpoint())),
        new WaitCommand(0.5), // reset for isAtSetpoint commands to level out
        new InstantCommand(() -> wrist.setDegrees(Constants.REV_WRIST_AMP_DEGREES), wrist),
        new WaitUntilCommand(() -> (shooter.isShooterAtSetpoint() && wrist.isAtSetpoint())),
        new WaitCommand(0.6), // Time for wrist to get to position
        new InstantCommand(
            () -> shooter.setFeederVoltage(Constants.Shooter.FEEDER_SHOOT_VOLTS),
            shooter), // Constants.FEEDER_FEEDFWD_VOLTS
        new WaitUntilCommand(
            () ->
                lineBreakDebouncer.calculate(!shooter.isCenterBroken())), // probably debounce this
        new InstantCommand(shooter::stopFeeder, shooter),
        // new WaitUntilCommand(() -> lineBreakDebouncer.calculate(shooter.isCenterBroken())),
        new InstantCommand(() -> wrist.setDegrees(35.0), shooter),
        new WaitCommand(0.06),
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
