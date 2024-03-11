// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReverseIntake extends SequentialCommandGroup {
  /** Creates a new ReverseIntake. */
  public ReverseIntake(Shooter shooter, Intake intake, Wrist wrist, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(shooter, intake, wrist, elevator);

    addCommands(
        new InstantCommand(
            () -> {
              shooter.setFeederVoltage(-ShooterConstants.FEEDER_FEEDFWD_VOLTS);
              intake.setVolts(-Constants.INTAKE_COLLECT_VOLTS);
              wrist.setDegrees(21); // testing
              elevator.goHome();
            },
            shooter,
            intake,
            wrist),
        new WaitCommand(3.0),
        new InstantCommand(
            () -> {
              shooter.stopFeeder();
              intake.stopCollecting();
            },
            shooter,
            intake));
  }
}
