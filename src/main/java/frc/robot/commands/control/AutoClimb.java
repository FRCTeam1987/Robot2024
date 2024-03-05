// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Util;
import frc.robot.commands.movement.SquareUpToAprilTag;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimb extends SequentialCommandGroup {
  /** Creates a new Climb. */
  public AutoClimb(Elevator Elevator, Climber Climber, Vision protonVision, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> Elevator.setLengthInches(Constants.ELEVATOR_TRAP_HEIGHT)),
        new WaitUntilCommand(
            () ->
                Util.isWithinTolerance(
                    Elevator.getLengthInches(), Constants.ELEVATOR_TRAP_HEIGHT, 3)),
        new SquareUpToAprilTag(drivetrain, protonVision, Constants.TRAP_APRILTAG_HEIGHT, 0.25),
        new GoToHeightElevator(Elevator, Constants.ELEVATOR_TRAP_COLLAPSED_HEIGHT),
        // Climber Subsystem needs to be integrated.
        new MoveGates(Climber, true));
  }
}
