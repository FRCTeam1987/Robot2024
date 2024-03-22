// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.control.amp.FireRevAmp;
import frc.robot.commands.control.amp.PrepRevAmp;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NewShootAmpAuto extends SequentialCommandGroup {
  /** Creates a new NewShootAmp. */
  public NewShootAmpAuto(Shooter shooter, Elevator elevator, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new PrepRevAmp(elevator, wrist),
        new WaitCommand(0.8),
        new FireRevAmp(shooter),
        new WaitCommand(0.1),
        new InstantCommand(() -> elevator.setLengthInches(4.2)));
  }
}
