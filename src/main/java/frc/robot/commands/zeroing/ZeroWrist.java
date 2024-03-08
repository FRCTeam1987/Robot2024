// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.zeroing;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroWrist extends SequentialCommandGroup {
  /** Creates a new ZeroElevator. */
  public ZeroWrist(Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(wrist);
    addCommands(
        new InstantCommand(() -> wrist.setVoltage(-3.0)),
        new WaitCommand(0.5),
        new WaitUntilCommand(() -> wrist.getVelocity() == 0.0),
        new InstantCommand(() -> wrist.setZero()),
        new InstantCommand(() -> wrist.stop()));
  }
}
