// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.amp;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireFwdAmp extends SequentialCommandGroup {
  /** Creates a new FireFwdAmp. */
  public FireFwdAmp(Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(shooter);
    addCommands(
        new InstantCommand(
            () -> shooter.setFeederVoltage(Constants.Shooter.FEEDER_FEEDFWD_VOLTS_AGRESSIVE)),
        new WaitUntilCommand(() -> !shooter.isCenterBroken()),
        new InstantCommand(() -> shooter.stopFeeder()),
        new InstantCommand(() -> shooter.stopShooter()));
  }
}
