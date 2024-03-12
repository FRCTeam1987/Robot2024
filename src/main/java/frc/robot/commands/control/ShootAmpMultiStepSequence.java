// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import java.util.function.BooleanSupplier;

public class ShootAmpMultiStepSequence extends Command {
  private static final double DEBOUNCE_TIME = 0.06;
  private final Elevator ELEVATOR;
  private final Wrist WRIST;
  private final Shooter SHOOTER;
  private final BooleanSupplier SHOULD_PROGRESS;
  private boolean isPrepped = false;
  private boolean isFinished = false;
  private Debouncer lineBreakDebouncer;

  /** Creates a new ShootAmpMultiStepSequence. */
  public ShootAmpMultiStepSequence(
      BooleanSupplier SHOULD_PROGRESS, Shooter SHOOTER, Elevator ELEVATOR, Wrist WRIST) {
    this.SHOOTER = SHOOTER;
    this.ELEVATOR = ELEVATOR;
    this.WRIST = WRIST;
    this.SHOULD_PROGRESS = SHOULD_PROGRESS;
    addRequirements(SHOOTER, ELEVATOR, WRIST);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lineBreakDebouncer = new Debouncer(DEBOUNCE_TIME);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!isPrepped) {
      System.out.println("prepping");
      new SequentialCommandGroup(
              new InstantCommand(
                  () -> {
                    ELEVATOR.setLengthInches(Constants.ELEVATOR_AMP_HEIGHT);
                  },
                  ELEVATOR),
              new WaitCommand(0.06), // reset for isAtSetpoint commands to level out
              new InstantCommand(() -> WRIST.setDegrees(Constants.WRIST_AMP_DEGREES), WRIST),
              new WaitUntilCommand(WRIST::isAtSetpoint).withTimeout(0.75))
          .schedule();
      isPrepped = true;
    } else {
      if (SHOULD_PROGRESS.getAsBoolean()) {
        System.out.println("progressing");
        new SequentialCommandGroup(
                new InstantCommand(
                    () -> SHOOTER.setRPMShootNoSpin(Constants.Shooter.SHOOTER_AMP_RPM), SHOOTER),
                new WaitUntilCommand(SHOOTER::isShooterAtSetpoint).withTimeout(2.0),
                // new WaitCommand(1.0), // Time for wrist to get to position
                new InstantCommand(
                    () -> SHOOTER.setFeederVoltage(Constants.Shooter.FEEDER_SHOOT_VOLTS),
                    SHOOTER), // Constants.FEEDER_FEEDFWD_VOLTS
                new WaitUntilCommand(() -> lineBreakDebouncer.calculate(!SHOOTER.isCenterBroken()))
                    .withTimeout(2.0), // probably debounce this
                new InstantCommand(SHOOTER::stopFeeder, SHOOTER),
                // new WaitUntilCommand(() ->
                // lineBreakDebouncer.calculate(shooter.isCenterBroken())),
                new InstantCommand(() -> WRIST.setDegrees(35.0), SHOOTER),
                new WaitCommand(0.2),
                new InstantCommand(
                    () -> {
                      SHOOTER.stopShooter();
                      ELEVATOR.goHome();
                      WRIST.setDegrees(25.0);
                    },
                    SHOOTER,
                    ELEVATOR,
                    WRIST))
            .schedule();
        isFinished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.get().reInitAmpScore();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
