// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.movement;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.control.PrepareShootAmp;
import frc.robot.commands.control.ShootAmp;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class squareUpAndShootAmp extends SequentialCommandGroup {

  /** Creates a new squareUpAndShootAmp. */
  public squareUpAndShootAmp(
      Vision photonVision, Drivetrain drivetrain, Wrist wrist, Elevator elevator, Shooter shooter) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new SquareUpToAprilTag(drivetrain, photonVision, Constants.AMP_APRILTAG_HEIGHT, 0.5),
            new PrepareShootAmp(elevator, wrist)),
        new WaitCommand(0.5), // Allow for systems to move.
        new WaitUntilCommand(() -> (shooter.isShooterAtSetpoint() && wrist.isAtSetpoint())),
        new ShootAmp(shooter, elevator, wrist));
  }
}
