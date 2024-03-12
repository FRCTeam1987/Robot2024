// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.movement.PointAtAprilTag;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.wrist.Wrist;

public class ShootNoteSequence extends SequentialCommandGroup {
  private static final double DEBOUNCE_TIME = 0.06;

  /** Creates a new IntakeNoteSequence. */
  private final Debouncer lineBreakDebouncer;

  public ShootNoteSequence(Shooter shooter, Wrist wrist, double shootRPM, double wristDegrees) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    lineBreakDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);

    addCommands(
        new InstantCommand(
            () -> {
              wrist.setDegrees(wristDegrees);
              shooter.setRPMShoot(shootRPM);
            },
            shooter,
            wrist),
        new WaitCommand(0.1), // reset for isAtSetpoint commands to level out
        new WaitUntilCommand(() -> wrist.isAtSetpoint() && shooter.isShooterAtSetpoint()),
        new WaitCommand(0.2), // Time for writst to get to position
        new InstantCommand(
            () -> shooter.setFeederVoltage(ShooterConstants.FEEDER_SHOOT_VOLTS),
            shooter), // Constants.FEEDER_FEEDFWD_VOLTS
        new WaitUntilCommand(
            () ->
                lineBreakDebouncer.calculate(!shooter.isCenterBroken())), // probably debounce this
        new InstantCommand(shooter::stopFeeder, shooter),
        new WaitUntilCommand(() -> lineBreakDebouncer.calculate(shooter.isCenterBroken())),
        new InstantCommand(shooter::stopShooter, shooter),
        new WaitCommand(0.1));
    // new InstantCommand(() -> wrist.goHome(), wrist));
  }

  public ShootNoteSequence(
      Shooter shooter, Wrist wrist, double shootRPM, Drivetrain drivetrain, Vision photonVision) {
    addRequirements(shooter);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    lineBreakDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);

    addCommands(
        new ParallelCommandGroup(
                new PointAtAprilTag(drivetrain, photonVision),
                new InstantCommand(
                    () -> {
                      // TODO: FIX ME !!!!!
                      // wrist.setDegrees(RobotContainer.get().getDouble(30));
                      shooter.setRPMShoot(shootRPM);
                    },
                    shooter,
                    wrist))
            .withTimeout(0.25),
        new WaitCommand(0.1), // reset for isAtSetpoint commands to level out
        new WaitUntilCommand(() -> wrist.isAtSetpoint() && shooter.isShooterAtSetpoint())
            .withTimeout(0.15),
        new WaitCommand(0.4), // Time for writst to get to position
        new InstantCommand(
            () -> shooter.setFeederVoltage(ShooterConstants.FEEDER_SHOOT_VOLTS),
            shooter), // Constants.FEEDER_FEEDFWD_VOLTS
        new WaitUntilCommand(() -> lineBreakDebouncer.calculate(!shooter.isCenterBroken()))
            .withTimeout(0.2), // probably debounce this
        new InstantCommand(shooter::stopFeeder, shooter),
        new WaitUntilCommand(() -> lineBreakDebouncer.calculate(shooter.isCenterBroken()))
            .withTimeout(0.2),
        new InstantCommand(shooter::stopShooter, shooter));
    // new InstantCommand(() -> wrist.goHome(), wrist));
  }

  public ShootNoteSequence(
      Shooter shooter,
      Wrist wrist,
      Elevator elevator,
      double shootRPM,
      double wristDegrees,
      double elevatorInches) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    lineBreakDebouncer = new Debouncer(DEBOUNCE_TIME, DebounceType.kFalling);

    addCommands(
        new InstantCommand(
            () -> {
              wrist.setDegrees(wristDegrees);
              shooter.setRPMShoot(shootRPM);
              elevator.setLengthInches(elevatorInches);
            },
            shooter,
            wrist),
        new WaitCommand(0.1), // reset for isAtSetpoint commands to level out
        new WaitUntilCommand(
            () -> wrist.isAtSetpoint() && shooter.isShooterAtSetpoint() && elevator.isAtSetpoint()),
        new WaitCommand(0.4), // Time for writst to get to position
        new InstantCommand(
            () -> shooter.setFeederVoltage(ShooterConstants.FEEDER_FEEDFWD_VOLTS),
            shooter), // Constants.FEEDER_FEEDFWD_VOLTS
        new WaitUntilCommand(
            () ->
                lineBreakDebouncer.calculate(!shooter.isCenterBroken())), // probably debounce this
        new InstantCommand(shooter::stopFeeder, shooter),
        new WaitUntilCommand(() -> lineBreakDebouncer.calculate(shooter.isCenterBroken())),
        new InstantCommand(
            () -> {
              shooter.stopShooter();
              elevator.goHome();
            },
            shooter),
        new WaitCommand(0.1));
    // new InstantCommand(() -> wrist.goHome(), wrist));
  }
}
