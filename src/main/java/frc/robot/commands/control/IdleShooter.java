// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.Shooter;

public class IdleShooter extends Command {
  /** Creates a new IdleShooter. */
  private Shooter shooter;

  private double DEBOUNCE_TIME = 2.0;
  private Debouncer debounce;

  public IdleShooter(Shooter shooter) {
    addRequirements(shooter);
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    debounce = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (shooter.isLineBreakBroken()) {
      if (debounce.calculate(
          shooter.ShooterCameraDistanceToTarget(Constants.SPEAKER_APRILTAG_HEIGHT) < 2.0)) {
        shooter.setRPMShoot(Constants.SHOOTER_IDLE_CLOSERANGE_RPM);
      } else {
        shooter.setRPMShoot(Constants.SHOOTER_IDLE_RPM);
      }
    // } else {
    //   shooter.stopShooter();
    // }
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
