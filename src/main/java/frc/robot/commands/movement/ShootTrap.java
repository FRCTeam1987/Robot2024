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
public class ShootTrap extends SequentialCommandGroup {
  /** Creates a new ShootTrap. */
  public ShootTrap(final Elevator elevator, final Wrist wrist, final Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(
            () -> {
              elevator.setLengthInches(8.0);
            },
            elevator),
        new WaitUntilCommand(() -> elevator.isAtSetpoint()),
        new InstantCommand(
            () -> {
              wrist.setDegrees(95);
              shooter.setRPMShoot(1500);
            },
            wrist,
            shooter),
        new WaitCommand(0.5),
        new WaitUntilCommand(() -> wrist.isAtSetpoint() && shooter.isShooterAtSetpoint()),
        new InstantCommand(() -> shooter.setFeederVoltage(6.0), shooter),
        new WaitUntilCommand(() -> !shooter.isLineBreakBroken()),
        new WaitCommand(0.2),
        new InstantCommand(
            () -> {
              wrist.setDegrees(30);
              shooter.stopShooter();
              shooter.stopFeeder();
            },
            wrist,
            shooter),
        new InstantCommand(
            () -> {
              elevator.setLengthInches(0.5);
              wrist.setDegrees(20);
            },
            elevator,
            wrist));
  }
}
