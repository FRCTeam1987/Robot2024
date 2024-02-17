// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

public class ShootNoteSequence extends SequentialCommandGroup {
  /** Creates a new IntakeNoteSequence. */
  public ShootNoteSequence(Shooter shooter, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(
            () -> {
              wrist.moveToPositionDegrees(35);
              shooter.setRPMShoot(1500);
            },
            shooter,
            wrist),
        new WaitUntilCommand(() -> wrist.isAtSetpoint() && shooter.isShooterAtSetpoint()),
        new InstantCommand(() -> shooter.setFeederVoltage(5), shooter),
        new WaitUntilCommand(() -> shooter.isLineBreakBroken()), // probably debounce this
        new InstantCommand(
            () -> {
              shooter.stopFeeder();
            },
            shooter),
        new WaitUntilCommand(() -> !shooter.isLineBreakBroken()),
        new InstantCommand(
            () -> {
              shooter.stopShooter();
            },
            shooter),
        new WaitUntilCommand(0.1),
        new InstantCommand(() -> wrist.moveToPositionDegrees(WristConstants.WRIST_MIN_DEG), wrist));
  }
}
