// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.climber.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveGates extends SequentialCommandGroup {
  public MoveGates(final Climber CLIMBER, final boolean direction) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(CLIMBER);
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(() -> CLIMBER.setSpeeds(direction ? Constants.CLIMBER_NOMINAL_VOLTAGE : -Constants.CLIMBER_NOMINAL_VOLTAGE)),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> CLIMBER.getLeftCurrent() > Constants.CLIMBER_CUTOFF_AMPERAGE),
                new InstantCommand(() -> CLIMBER.stopLeft(direction))),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> CLIMBER.getRightCurrent() > Constants.CLIMBER_CUTOFF_AMPERAGE),
                new InstantCommand(() -> CLIMBER.stopRight(direction)))));
  }
}
