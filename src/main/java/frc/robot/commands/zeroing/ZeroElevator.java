// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.zeroing;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroElevator extends SequentialCommandGroup {
  /** Creates a new ZeroElevator. */
  public ZeroElevator(Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(elevator);
    addCommands(
        new InstantCommand(() -> elevator.setVoltage(-2.0)),
        new WaitCommand(0.5),
        new WaitUntilCommand(() -> elevator.getVelocity() == 0.0),
        new InstantCommand(() -> elevator.zeroPosition()),
        new InstantCommand(() -> elevator.stop()));
  }
}
