// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class SpitNote extends SequentialCommandGroup {
  /** Creates a new IntakeNoteSequence. */
  public final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("SHOOTER");

  final GenericEntry SpitRPM = SHOOTER_TAB.add("SpitRPM", 3500).getEntry();

  public SpitNote(Shooter shooter) {
    addRequirements(shooter);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        new InstantCommand(() -> shooter.setRPMShoot(SpitRPM.getDouble(900)), shooter),
        new WaitCommand(0.1), // reset for isAtSetpoint commands to level out
        new WaitUntilCommand(shooter::isShooterAtSetpoint),
        new WaitCommand(1.0), // reset for isAtSetpoint commands to level out
        new InstantCommand(
            () -> shooter.setFeederVoltage(ShooterConstants.FEEDER_FEEDFWD_VOLTS),
            shooter), // Constants.FEEDER_FEEDFWD_VOLTS
        new WaitCommand(1.0), // reset for isAtSetpoint commands to level out
        new InstantCommand(shooter::stopFeeder, shooter),
        new InstantCommand(shooter::stopShooter, shooter));
  }
}
