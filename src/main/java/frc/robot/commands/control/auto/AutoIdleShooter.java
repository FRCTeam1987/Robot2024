// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

public class AutoIdleShooter extends Command {

  private static final double VALID_SHOT_DEBOUNCE_TIME = 0.2;

  /** Creates a new IdleShooter. */
  private final Shooter shooter;

  // private final Debouncer validShotDebouncer;

  public AutoIdleShooter(Shooter shooter) {
    addRequirements(shooter);
    this.shooter = shooter;
    // this.validShotDebouncer = new Debouncer(VALID_SHOT_DEBOUNCE_TIME, DebounceType.kFalling);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (shooter.isCenterBroken()) {
    //   RobotContainer.aimAtTargetAuto = true;
    // } else {
    //   RobotContainer.aimAtTargetAuto = false;
    // }
    // if (shooter.isCenterBroken() &&
    // validShotDebouncer.calculate(Util.isValidShot(SPEAKER_LIMELIGHT))) {
    shooter.setRPMShoot(Constants.Shooter.SHOOTER_RPM);
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
