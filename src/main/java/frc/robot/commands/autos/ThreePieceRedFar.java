// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Red Far (Audience Side)
//    1. Shoot pre-load
//    2. Drive to far note
//    3. Pick it up and shoot
//    4. Drive to middle note
//    5. Pick it up and shoot

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceRedFar extends SequentialCommandGroup {
  /** Creates a new ThreePieceRedFar. */
  public ThreePieceRedFar() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
