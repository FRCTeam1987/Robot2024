// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

public class AutoIdleShooter extends Command {

  private static final double VALID_SHOT_DEBOUNCE_TIME = 0.2;
  private static final double POOP_RPM = 550;

  /** Creates a new IdleShooter. */
  private final Shooter shooter;

  private final Supplier<AutoState> autoStateSupplier;

  // private final Debouncer validShotDebouncer;

  public AutoIdleShooter(Shooter shooter) {
    this(shooter, () -> null);
  }

  public AutoIdleShooter(final Shooter shooter, final Supplier<AutoState> autoStateSupplier) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.autoStateSupplier = autoStateSupplier;
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
    final AutoState state = autoStateSupplier.get();
    switch (state) {
      case COLLECTING:
        if (shooter.isCenterBroken()) {
          shooter.stopFeeder();
        } else {
          shooter.setFeederVoltage(Constants.Shooter.FEEDER_FEEDFWD_VOLTS);
        }
        final double poseX = RobotContainer.DRIVETRAIN.getPose().getX();
        if (poseX < 6.0 || poseX > 16.56 - 6.0) {
          shooter.setRPMShoot(Constants.Shooter.SHOOTER_RPM);
        } else {
          shooter.setRPMShootNoSpin(POOP_RPM);
        }
        break;
      case POOP_PREP:
        shooter.stopFeeder();
        shooter.setRPMShootNoSpin(POOP_RPM);
        break;
      case POOPING:
        shooter.setFeederVoltage(Constants.Shooter.FEEDER_FEEDFWD_VOLTS);
        shooter.setRPMShootNoSpin(POOP_RPM);
        break;
      case SHOOT_PREP:
        if (shooter.isRearBroken() && !shooter.isCenterBroken()) {
          shooter.setFeederVoltage(Constants.Shooter.FEEDER_FEEDFWD_VOLTS);
        } else {
          shooter.stopFeeder();
        }
        shooter.setRPMShoot(Constants.Shooter.SHOOTER_RPM);
        break;
      case SHOOTING:
        shooter.setFeederVoltage(Constants.Shooter.FEEDER_AUTO_VOLTS + 2.0);
        shooter.setRPMShoot(Constants.Shooter.SHOOTER_RPM);
        break;
      default:
        shooter.setRPMShoot(Constants.Shooter.SHOOTER_RPM);
        if (shooter.isCenterBroken()) {
          shooter.stopFeeder();
          if (state != AutoState.SHOOTING && state != AutoState.SHOOT_PREP) {
            RobotContainer.setAutoState(AutoState.SHOOT_PREP);
          }
        } else if (shooter.isRearBroken() && !shooter.isCenterBroken()) {
          shooter.setFeederVoltage(Constants.Shooter.FEEDER_FEEDFWD_VOLTS);
        } else {
          shooter.stopFeeder();
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
