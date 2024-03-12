// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class Rumble extends Command {
  private final XboxController controller;
  private final double strength;
  private final RumbleType type;
  private long finishtime;
  private final long durationMs;

  /** Creates a new Rumble. */
  public Rumble(XboxController controller, RumbleType type, double strength, long durationMs) {
    this.controller = controller;
    this.strength = strength;
    this.durationMs = durationMs;
    this.type = type;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    long time = System.currentTimeMillis();
    this.finishtime = time + durationMs;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.setRumble(type, strength);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.setRumble(RumbleType.kBothRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() >= finishtime;
  }
}
