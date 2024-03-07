// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

public class PrepareShootAmp extends SequentialCommandGroup {
  /** Creates a new IntakeNoteSequence. */
  public PrepareShootAmp(Shooter shooter, Elevator elevator, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        new InstantCommand(
            () -> {
              shooter.setRPMShoot(Constants.SHOOTER_AMP_RPM);
              elevator.setLengthInches(Constants.ELEVATOR_AMP_HEIGHT);
              // wrist.setDegrees(Constants.WRIST_AMP_DEGREES);  // 25.0 + 90.0
            },
            shooter,
            elevator),
        new WaitCommand(0.5), // reset for isAtSetpoint commands to level out
        new InstantCommand(() -> wrist.setDegrees(Constants.WRIST_AMP_DEGREES), wrist),
        new WaitUntilCommand(() -> (shooter.isShooterAtSetpoint() && wrist.isAtSetpoint())));
  }
}
