// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb extends SequentialCommandGroup {
  /** Creates a new Climb. */
  public Climb(Elevator Elevator, Climber Climber) {
    addRequirements(Elevator, Climber);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new ConditionalCommand(
        // new GoToHeightElevator(Elevator, Constants.ELEVATOR_TRAP_COLLAPSED_HEIGHT),
        new InstantCommand(
            () -> Elevator.setLengthInches(Constants.ELEVATOR_TRAP_COLLAPSED_HEIGHT)),
        new WaitCommand(2.0),
        new MoveGates(Climber, true));

    // new InstantCommand(
    //     () -> {
    //       System.out.println("Climb command left, Climber Location wrong");
    //       return;
    //     }),
    // () ->
    //     Util.isWithinTolerance(
    //         Elevator.getLengthInches(), Constants.ELEVATOR_TRAP_HEIGHT, 0.5)));
    // Climber Subsystem needs to be integrated.
  }
}
