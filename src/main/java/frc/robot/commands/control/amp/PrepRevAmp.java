// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.amp;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepRevAmp extends SequentialCommandGroup {
  /** Creates a new PrepFwdAmp. */
  public PrepRevAmp(Elevator elevator, Wrist wrist) {
    addRequirements(elevator, wrist);

    addCommands(
        new InstantCommand(
            () -> {
              elevator.setLengthInches(Constants.REV_ELEVATOR_AMP_HEIGHT);
              wrist.setDegrees(Constants.REV_WRIST_AMP_DEGREES);
            },
            elevator,
            wrist),
        new WaitUntilCommand(() -> wrist.isAtSetpoint() && elevator.isAtSetpoint())
            .withTimeout(0.75));
  }
}
