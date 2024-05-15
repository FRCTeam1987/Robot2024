// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Wrist;
import frc.robot.util.Util;

public class JiggleTrap extends Command {
  private boolean doTheJiggle;
  private final Wrist WRIST;
  /** Creates a new JiggleTrap. */
  public JiggleTrap(Wrist wrist) {
    this.WRIST = wrist;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    doTheJiggle = Util.isWithinTolerance(WRIST.getDegrees(), Constants.Trap.TRAP_WRIST_DEGREES, 3.0);
    if (doTheJiggle) {
      WRIST.setDegreesSlot1(Constants.Trap.TRAP_WRIST_DEGREES - 7.0);
    } else {
      WRIST.setDegreesSlot1(Constants.Trap.TRAP_WRIST_DEGREES);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
