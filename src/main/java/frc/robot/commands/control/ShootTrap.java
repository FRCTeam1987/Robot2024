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
public class ShootTrap extends SequentialCommandGroup {

  private final double elevatorHeight = 29.5;
  private final double wristDegrees = 112.5;
  private final double rpmSpeed = 525;

  private final Debouncer lineBreakDebouncer;

  private static final double DEBOUNCE_TIME = 0.06;

  /** Creates a new ShootTrap. */
  public ShootTrap(final Elevator elevator, final Wrist wrist, final Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    lineBreakDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);

    addCommands(
        new InstantCommand(
            () -> {
              shooter.setRPMShoot(rpmSpeed);
              elevator.setLengthInches(elevatorHeight);
              shooter.setFeederVoltage(-0.2);
              ;
              // wrist.setDegrees(110.0);  // 25.0 + 90.0
            },
            shooter,
            elevator),
        // new WaitUntilCommand(() -> (elevator.isAtSetpoint())),
        new WaitUntilCommand(() -> elevator.isAtSetpoint()),
        new WaitCommand(0.8), // reset for isAtSetpoint commands to level out
        new InstantCommand(() -> wrist.setDegrees(wristDegrees), wrist),
        new WaitUntilCommand(() -> (shooter.isShooterAtSetpoint() && wrist.isAtSetpoint())),
        new WaitCommand(1.4), // Time for wrist to get to position
        new InstantCommand(
            () -> shooter.setFeederVoltage(ShooterConstants.FEEDER_SHOOT_VOLTS),
            shooter), // Constants.FEEDER_FEEDFWD_VOLTS
        new WaitUntilCommand(
            () ->
                lineBreakDebouncer.calculate(!shooter.isCenterBroken())), // probably debounce this
        new InstantCommand(shooter::stopFeeder, shooter));
    // new WaitUntilCommand(() -> lineBreakDebouncer.calculate(shooter.isCenterBroken())),
    // new InstantCommand(() -> wrist.setDegrees(35.0), shooter),
    // new WaitCommand(0.4),
    // new InstantCommand(
    //     () -> {
    //       shooter.stopShooter();
    //       elevator.goHome();
    //       wrist.setDegrees(25.0);
    //     },
    //     shooter,
    //     elevator,
    //     wrist));
  }
}
