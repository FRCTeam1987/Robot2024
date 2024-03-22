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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootTrap extends SequentialCommandGroup {
  private final Debouncer lineBreakDebouncer;

  /** Creates a new ShootTrap. */
  public ShootTrap(final Elevator elevator, final Wrist wrist, final Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    lineBreakDebouncer = new Debouncer(Constants.Trap.TRAP_DEBOUNCE_TIME, DebounceType.kFalling);

    addCommands(
        new InstantCommand(
            () -> elevator.setLengthInches(Constants.Trap.TRAP_ELEVATOR_HEIGHT_MIDWAY)),
        new WaitUntilCommand(() -> elevator.isAtSetpoint()),
        new WaitCommand(0.5),
        new InstantCommand(() -> wrist.setDegrees(Constants.Trap.TRAP_WRIST_DEGREES_MIDWAY)),
        new WaitUntilCommand(() -> wrist.isAtSetpoint()),
        new WaitCommand(0.5),
        new InstantCommand(
            () -> {
              shooter.setRPMShoot(Constants.Trap.TRAP_RPM_SPEED);
              elevator.setLengthInches(Constants.Trap.TRAP_ELEVATOR_HEIGHT);
              shooter.setFeederVoltage(-0.2);
              // wrist.setDegrees(110.0);  // 25.0 + 90.0
            },
            shooter,
            elevator),
        // new WaitUntilCommand(() -> (elevator.isAtSetpoint())),
        new InstantCommand(() -> wrist.setDegreesSlot1(Constants.Trap.TRAP_WRIST_DEGREES), wrist),
        new WaitUntilCommand(
                () ->
                    (shooter.isShooterAtSetpoint()
                        && wrist.isAtSetpoint()
                        && elevator.isAtSetpoint()))
            .withTimeout(0.8),
        new WaitCommand(0.8), // Time for wrist to get to position
        new InstantCommand(
            () -> shooter.setFeederVoltage(Constants.Shooter.FEEDER_SHOOT_VOLTS),
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
