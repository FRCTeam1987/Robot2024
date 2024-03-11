// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopAll extends ParallelCommandGroup {
  /** Creates a new StopALl. */
  public StopAll(Wrist wrist, Shooter shooter, Intake intake, Elevator elevator) {
    addRequirements(shooter, intake, wrist, elevator);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(shooter::stopShooter),
        new InstantCommand(shooter::stopFeeder),
        new InstantCommand(intake::stopCollecting),
        new InstantCommand(wrist::stop),
        // new InstantCommand(() -> climber.stopAll()),
        new InstantCommand(elevator::zeroVoltage),
        new InstantCommand(elevator::coastElevator));
  }
}
