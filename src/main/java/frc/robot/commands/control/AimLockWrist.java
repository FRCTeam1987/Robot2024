// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.InterpolatingDouble;

public class AimLockWrist extends Command {
  private Wrist wrist;
  private Shooter shooter;
  private Elevator elevator;

  /** Creates a new AimLockWrist. */
  public AimLockWrist(Wrist wrist, Shooter shooter, Elevator elevator) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.shooter = shooter;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.SPEAKER_PROTON.hasTargets()) {
      final double ty = Constants.SPEAKER_PROTON.getPitchVal();
      if (RobotContainer.get().SHOOTER.isCenterBroken() && (ty > 2 || ty < 4)) {
        // try {
        double distance =
            Vision.calculateDistanceToTarget(
                Constants.SPEAKER_PROTON.getPitchVal(),
                Constants.SPEAKER_PROTON_HEIGHT,
                Constants.SPEAKER_APRILTAG_HEIGHT,
                Constants.SPEAKER_PROTON_ANGLE);
        System.out.println("Calculating for: " + distance);
        double degrees = 0.0;
        if (shooter.ShooterCameraDistanceToTarget(Constants.SPEAKER_APRILTAG_HEIGHT) < 2.0
            && elevator.getLengthInches() > 9.0) {
          degrees =
              Constants.DISTANCE_WRIST_ANGLE_MAP_ELEVATOR.getInterpolated(
                      new InterpolatingDouble(distance))
                  .value;
          System.out.println("Degrees attempted: " + degrees);
          wrist.setDegrees(degrees);
        } else if (shooter.ShooterCameraDistanceToTarget(Constants.SPEAKER_APRILTAG_HEIGHT) > 2.0) {
          degrees =
              Constants.DISTANCE_WRIST_ANGLE_MAP_NONELEVATOR.getInterpolated(
                      new InterpolatingDouble(distance))
                  .value;
          System.out.println("Degrees attempted: " + degrees);
          wrist.setDegrees(degrees);
        }

        // if (degrees < 21 || degrees > 42) {
        //   DriverStation.reportError("WristAim Map Out of Range", false);
        // } else {

        // }
        // } catch (Exception ignored) {
        // }

      } else {
        wrist.goHome();
      }
    } else {
      // DriverStation.reportError("Cannot see apriltag", false);
      return;
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
