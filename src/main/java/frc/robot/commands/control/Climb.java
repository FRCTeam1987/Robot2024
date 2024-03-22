// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb extends SequentialCommandGroup {
  /** Creates a new Climb. */
  public Climb(Elevator Elevator, Wrist Wrist, Shooter Shooter) {
    addRequirements(Elevator, Wrist, Shooter);
    addCommands(
        new InstantCommand(Shooter::stopShooter, Shooter),
        new InstantCommand(() -> Shooter.setFeederVoltage(-0.2)),
        new InstantCommand(Wrist::goHome, Wrist),
        new InstantCommand(Wrist::stop, Wrist),
        new InstantCommand(() -> Wrist.setDegrees(Constants.Wrist.INITIAL_ANGLE_DEGREES), Wrist),
        new InstantCommand(
            () -> Elevator.setLengthInchesSlot1(Constants.Climb.CLIMB_PULLDOWN_HEIGHT)),
        new WaitUntilCommand(Elevator::isAtSetpoint),
        new WaitCommand(0.9),
        new InstantCommand(() -> Elevator.setLengthInches(Constants.Climb.CLIMB_LEVEL_HEIGHT)),
        new WaitUntilCommand(Elevator::isAtSetpoint),
        new WaitCommand(2.0),
        new InstantCommand(Wrist::goHome, Wrist),
        new InstantCommand(Wrist::stop, Wrist),
        new InstantCommand(() -> Wrist.setDegrees(Constants.Wrist.INITIAL_ANGLE_DEGREES), Wrist),
        new ConditionalCommand(
            new ShootTrap(Elevator, Wrist, Shooter),
            new InstantCommand(() -> System.out.println("No Note deteced. Climb finished.")),
            Shooter::isCenterBroken));
  }
}
