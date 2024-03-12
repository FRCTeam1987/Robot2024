// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class IdleShooter extends Command {
  /** Creates a new IdleShooter. */
  private final Shooter shooter;

  public IdleShooter(Shooter shooter) {
    addRequirements(shooter);
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double DEBOUNCE_TIME = 2.0;
    Debouncer debounce = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.isClimbPrimed) {
      shooter.stopShooter();
      return;
    }
    if (RobotContainer.isAmpPrepped) {
      shooter.setRPMShoot(ShooterConstants.SHOOTER_AMP_RPM);
      return;
    }
    if (shooter.isCenterBroken()) {
      shooter.setRPMShoot(ShooterConstants.SHOOTER_IDLE_RPM);
    } else {
      shooter.stopShooter();
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
