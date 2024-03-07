// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Util;
import frc.robot.subsystems.elevator.Elevator;

public class GoToHeightElevator extends Command {
  /** Creates a new GoToHeightElevator. */
  private Elevator elevator;

  private double heightInches = 1;

  public GoToHeightElevator(Elevator elevator, double heightInches) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.heightInches = heightInches;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new InstantCommand(() -> elevator.setLengthInches(heightInches));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Util.isWithinTolerance(elevator.getLengthInches(), heightInches, 0.1);
  }
}
