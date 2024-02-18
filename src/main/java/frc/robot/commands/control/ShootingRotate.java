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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

public class ShootingRotate extends SequentialCommandGroup {
  /** Creates a new IntakeNoteSequence. */
  private Debouncer lineBreakDebouncer;

  private static final double DEBOUNCE_TIME = 0.06;

  public ShootingRotate(Shooter shooter, Wrist wrist, double shootRPM, double wristDegrees) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    lineBreakDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);

    addCommands(
        // new PointAtAprilTag(drivetrain, null, getName())
        new InstantCommand(
            () -> {
              wrist.moveToPositionDegrees(RobotContainer.SHOOT_ANGLE.getDouble(30));
              shooter.setRPMShoot(shootRPM);
            },
            shooter,
            wrist),
        new WaitCommand(0.1), // reset for isAtSetpoint commands to level out
        new WaitUntilCommand(() -> wrist.isAtSetpoint() && shooter.isShooterAtSetpoint()),
        new WaitCommand(0.5), // Time for writst to get to position
        new InstantCommand(
            () -> shooter.setFeederVoltage(14), shooter), // Constants.FEEDER_FEEDFWD_VOLTS
        new WaitUntilCommand(
            () ->
                lineBreakDebouncer.calculate(
                    shooter.isLineBreakBroken())), // probably debounce this
        new InstantCommand(
            () -> {
              shooter.stopFeeder();
            },
            shooter),
        new WaitUntilCommand(() -> !lineBreakDebouncer.calculate(shooter.isLineBreakBroken())),
        new InstantCommand(
            () -> {
              shooter.stopShooter();
            },
            shooter),
        new WaitCommand(0.1),
        new InstantCommand(() -> wrist.moveToPositionRotations(0), wrist));
  }
}
