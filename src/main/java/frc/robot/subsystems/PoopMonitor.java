// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class PoopMonitor extends SubsystemBase {

  private boolean hasPooped = false;
  private boolean shouldMonitor = false;

  public boolean hasPooped() {
    return hasPooped;
  }

  public void resetPooped() {
    hasPooped = false;
  }

  public void startMonitor() {
    shouldMonitor = true;
  }

  public void stopMonitor() {
    shouldMonitor = false;
  }

  public Command StartPoopMonitorCommand() {
    return new InstantCommand(() -> startMonitor());
  }

  public Command StopPoopMonitorCommand() {
    return new InstantCommand(() -> stopMonitor());
  }

  public Command ResetPoopMonitorCommand() {
    return new InstantCommand(() -> resetPooped());
  }

  /** Creates a new PoopMonitor. */
  public PoopMonitor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (shouldMonitor && !hasPooped && RobotContainer.SHOOTER.hasNote()) {
      hasPooped = true;
      // DriverStation.reportWarning("Pooped a note!", false);
    }
  }
}
