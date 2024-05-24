// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Util;

public class IdleShooter extends Command {
  /** Creates a new IdleShooter. */
  private final Shooter shooter;

  private final Debouncer validShotDebouncer;

  public IdleShooter(Shooter shooter) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.validShotDebouncer = new Debouncer(3.0, DebounceType.kFalling);
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
      shooter.setRPMShoot(Constants.Shooter.SHOOTER_AMP_RPM);
      return;
    }
    if (shooter.isCenterBroken() || shooter.isRearBroken()) {
        shooter.setRPMShoot(Util.getShooterSpeedFromDistanceForLob(Util.getDistanceToAllianceLob(RobotContainer.get().getPose())));
    } else {
      if (validShotDebouncer.calculate(Util.isValidShotAmpInclusive())) {
        shooter.setRPMShoot(Util.getShooterSpeedFromDistanceForLob(Util.getDistanceToAllianceLob(RobotContainer.get().getPose())));
      } else {
        shooter.stopShooter();
      }
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
