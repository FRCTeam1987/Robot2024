// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.movement.ShootTrap;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb extends SequentialCommandGroup {
  /** Creates a new Climb. */
  public Climb(Elevator Elevator, Wrist Wrist, Shooter Shooter) {
    addRequirements(Elevator, Wrist, Shooter);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new ConditionalCommand(
        // new GoToHeightElevator(Elevator, Constants.ELEVATOR_TRAP_COLLAPSED_HEIGHT),
        new InstantCommand(Shooter::stopShooter, Shooter),
        new InstantCommand(Wrist::stop, Wrist),
        new InstantCommand(() -> Elevator.setLengthInchesSlot1(5.9)),
        new WaitCommand(1.0),
        new WaitUntilCommand(Elevator::isAtSetpoint),
        new InstantCommand(Elevator::stop),
        // new WaitCommand(0.3),
        // new WaitUntilCommand(
        //     () ->
        //         Util.isWithinTolerance(
        //             Elevator.getLengthInches(), Constants.ELEVATOR_TRAP_COLLAPSED_HEIGHT, 0.07)),
        // new WaitCommand(0.02),
        new ConditionalCommand(
            new ShootTrap(Elevator, Wrist, Shooter),
            new InstantCommand(() -> System.out.println("No Note deteced. Climb finished.")),
            Shooter::isCenterBroken)
        //
        );

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
