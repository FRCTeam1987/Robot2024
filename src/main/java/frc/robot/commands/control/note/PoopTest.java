// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.note;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PoopTest extends SequentialCommandGroup {
  /** Creates a new PoopTest. */
  public PoopTest(Shooter shooter, Intake intake, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double POOP_RPM = 700; // 550s (CHANGE IN AUTOIDLESHOOTERTOO!!)
    addRequirements(shooter, intake, wrist);
    addCommands(
        new InstantCommand(
            () -> {
              intake.setVolts(-8.0);
              shooter.stopFeeder();
              shooter.setRPMShootNoSpin(POOP_RPM);
            }),
        new WaitCommand(1),
        new InstantCommand(
            () -> {
              shooter.setFeederVoltage(Constants.Shooter.FEEDER_FEEDFWD_VOLTS);
              shooter.setRPMShootNoSpin(POOP_RPM);
              wrist.setDegrees(22);
            }));
  }
}
