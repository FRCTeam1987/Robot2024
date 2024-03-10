// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.movement;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSubwooferFlat extends SequentialCommandGroup {
  /** Creates a new ShootTrap. */
  public ShootSubwooferFlat(final Elevator elevator, final Wrist wrist, final Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(
            () -> {
              elevator.setLengthInches(6.5);
              shooter.setRPMShoot(2750);
            },
            elevator,
            shooter),
        new WaitUntilCommand(elevator::isAtSetpoint),
        new InstantCommand(() -> wrist.setDegrees(50), wrist),
        new WaitCommand(0.44),
        new WaitUntilCommand(wrist::isAtSetpoint),
        new InstantCommand(() -> shooter.setFeederVoltage(6.0), shooter),
        new WaitUntilCommand(() -> !shooter.isCenterBroken()),
        new WaitCommand(0.04),
        new InstantCommand(
            () -> {
              elevator.setLengthInches(0.5);
              wrist.setDegrees(12);
              shooter.stopShooter();
              shooter.stopFeeder();
            },
            wrist,
            shooter));
  }
}
