// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleShootforAmp extends SequentialCommandGroup {
  /** Creates a new SimpleShootforAmp. */
  private Debouncer lineBreakDebouncer;

  private static final double DEBOUNCE_TIME = 0.06;

  public SimpleShootforAmp(Shooter shooter, Elevator elevator, Wrist wrist) {

    lineBreakDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);
    addRequirements(shooter, wrist, elevator);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> shooter.setRPMShootNoSpin(Constants.SHOOTER_AMP_RPM), shooter),
        new WaitUntilCommand(() -> (shooter.isShooterAtSetpoint())).withTimeout(2.0),
        // new WaitCommand(1.0), // Time for wrist to get to position
        new InstantCommand(
            () -> shooter.setFeederVoltage(Constants.FEEDER_SHOOT_VOLTS),
            shooter), // Constants.FEEDER_FEEDFWD_VOLTS
        new WaitUntilCommand(() -> lineBreakDebouncer.calculate(!shooter.isCenterBroken()))
            .withTimeout(2.0), // probably debounce this
        new InstantCommand(
            () -> {
              shooter.stopFeeder();
            },
            shooter),
        // new WaitUntilCommand(() -> lineBreakDebouncer.calculate(shooter.isCenterBroken())),
        new InstantCommand(() -> wrist.setDegrees(35.0), shooter),
        new WaitCommand(0.2),
        new InstantCommand(
            () -> {
              shooter.stopShooter();
              elevator.goHome();
              wrist.setDegrees(25.0);
              RobotContainer.setAmpPrepped(false);
            },
            shooter,
            elevator,
            wrist));
  }
}
