// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.note;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

public class PoopNote extends SequentialCommandGroup {
  /** Creates a new IntakeNoteSequence. */
  public PoopNote(Shooter shooter, double poopRPM) {
    addRequirements(shooter);
    addCommands(
        new InstantCommand(() -> shooter.setRPMShoot(poopRPM), shooter),
        new WaitCommand(0.1),
        new WaitUntilCommand(shooter::isShooterAtSetpoint).withTimeout(0.12),
        new InstantCommand(
            () -> shooter.setFeederVoltage(Constants.Shooter.FEEDER_SHOOT_VOLTS),
            shooter),
        new WaitUntilCommand(() -> !shooter.isCenterBroken()),
        new InstantCommand(shooter::stopFeeder, shooter));
  }
}
