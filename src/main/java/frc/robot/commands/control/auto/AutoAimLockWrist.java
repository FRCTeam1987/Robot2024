// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.util.Util;
import java.util.function.Supplier;

public class AutoAimLockWrist extends Command {
  private final Wrist wrist;
  private final Supplier<AutoState> autoStateSupplier;

  /** Creates a new AimLockWrist. */
  public AutoAimLockWrist(Wrist wrist) {
    this(wrist, () -> null);
  }

  public AutoAimLockWrist(final Wrist wrist, final Supplier<AutoState> autoStateSupplier) {
    addRequirements(wrist);
    this.wrist = wrist;
    this.autoStateSupplier = autoStateSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (autoStateSupplier.get()) {
      case COLLECTING:
        wrist.setDegrees(26);
        break;
      case POOPING:
        wrist.setDegrees(22);
        break;
      case SHOOT_PREP:
      case SHOOTING:
      case DEFAULT:
      default:
        wrist.setDegrees(MathUtil.clamp(Util.getInterpolatedWristAngleSpeaker(), 10.0, 35.0));
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
