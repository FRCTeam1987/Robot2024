// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.amp;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepFwdAmp extends SequentialCommandGroup {
  /** Creates a new PrepFwdAmp. */
  public PrepFwdAmp(Elevator elevator, Wrist wrist, Shooter shooter) {
    addRequirements(elevator, wrist, shooter);

    addCommands(
        new InstantCommand(
            () -> {
              elevator.setLengthInches(Constants.FWD_ELEVATOR_AMP_HEIGHT);
              wrist.setDegrees(Constants.FWD_WRIST_AMP_DEGREES);
              shooter.setRPMShootNoSpin(Constants.Shooter.SHOOTER_AMP_RPM);
            },
            elevator,
            wrist),
        new WaitUntilCommand(
                () ->
                    wrist.isAtSetpoint()
                        && elevator.isAtSetpoint()
                        && shooter.isShooterAtSetpoint())
            .withTimeout(0.75));
  }
}
