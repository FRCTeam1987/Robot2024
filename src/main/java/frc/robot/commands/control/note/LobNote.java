// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.note;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.Util;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LobNote extends SequentialCommandGroup {
  private final Debouncer lineBreakDebouncer;

  private final double DEBOUNCE_TIME = 0.08;

  // Add your commands in the addCommands() call, e.g.
  // addCommands(new FooCommand(), new BarCommand());

  /** Creates a new LobNote. */
  public LobNote(Shooter SHOOTER, Wrist WRIST, Elevator ELEVATOR, GenericEntry LOB_RPM) {

    lineBreakDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);
    addRequirements(SHOOTER, WRIST, ELEVATOR);
    addCommands(
        new InstantCommand(
            () -> {
              double dist = Util.getDistanceToAllianceLob(RobotContainer.get().getPose());
              WRIST.setDegrees(35.0);
              if (dist > 7.5) {
              SHOOTER.setRPMShoot(Util.getShooterSpeedFromDistanceForLob(dist));
              } else {
              SHOOTER.setRPMShoot(1800.0);
              }
              ELEVATOR.setLengthInches(0);
            },
            SHOOTER,
            WRIST),
        new WaitCommand(0.1),
        new WaitUntilCommand(
            () -> {
              boolean good = WRIST.isAtSetpoint() && SHOOTER.isShooterAtSetpoint();
              System.out.println("WRIST: " + WRIST.isAtSetpoint());
              System.out.println("SHOOTER: " + SHOOTER.isShooterAtSetpoint());
              return good;}),
        new WaitCommand(0.4),
        new InstantCommand(
            () -> SHOOTER.setFeederVoltage(Constants.Shooter.FEEDER_FEEDFWD_VOLTS_AGRESSIVE),
            SHOOTER),
        new WaitUntilCommand(() -> lineBreakDebouncer.calculate(!SHOOTER.isCenterBroken())),
        new InstantCommand(SHOOTER::stopFeeder, SHOOTER),
        new WaitUntilCommand(() -> lineBreakDebouncer.calculate(SHOOTER.isCenterBroken())),
        new InstantCommand(
            () -> {
              SHOOTER.stopShooter();
              ELEVATOR.goHome();
            },
            SHOOTER),
        new WaitCommand(0.1));
  }
}
