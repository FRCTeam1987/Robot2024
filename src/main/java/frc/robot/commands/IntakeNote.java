// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

public class IntakeNote extends Command {

  private final Shooter SHOOTER;
  private final Intake INTAKE;
  private final Wrist WRIST;
  private boolean hasBreaked;
  private boolean finished;
  private boolean is2ndstate;
  /** Creates a new IntakeNote. */
  public IntakeNote(Shooter shooter, Wrist wrist, Intake intake) {
    this.SHOOTER = shooter;
    this.WRIST = wrist;
    this.INTAKE = intake;
    addRequirements(shooter, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hasBreaked = SHOOTER.isLineBreakBroken();

    if (!hasBreaked) {
      if (is2ndstate) {
      SHOOTER.setFeederVoltage(0);
      WRIST.stop(); //magic number, study docs later. this is home.
      finished = true;
      }
      WRIST.moveToPositionRotations(2);
      SHOOTER.setFeederVoltage(9);
      INTAKE.setRPM(1200);
    } else {
      is2ndstate = true;
      INTAKE.setRPM(0);
      SHOOTER.setFeederVoltage(-4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
