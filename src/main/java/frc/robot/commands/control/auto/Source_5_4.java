// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Source_5_4 extends ParallelCommandGroup {

  private static final Pose2d BLUE_STARTING_POSE =
      PathPlannerAuto.getStaringPoseFromAutoFile("Source 5-4 Initial");
  private static final Pose2d RED_STARTING_POSE =
      new Pose2d(
          16.56 - BLUE_STARTING_POSE.getX(),
          BLUE_STARTING_POSE.getY(),
          BLUE_STARTING_POSE.getRotation().plus(Rotation2d.fromDegrees(180.0)));

  /** Creates a new Source_5_4. */
  public Source_5_4() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  RobotContainer.DRIVETRAIN.seedFieldRelative(
                      RobotContainer.DRIVETRAIN.getAlliance().equals(Alliance.Red)
                          ? RED_STARTING_POSE
                          : BLUE_STARTING_POSE);
                  RobotContainer.setAutoState(AutoState.DEFAULT);
                }),
            AutoBuilder.buildAuto("Source 5-4 AB"), // Source 5-4 Initial
            new ConditionalCommand(
                AutoBuilder.buildAuto("Source 5-4 B 5Y4Y"),
                new ConditionalCommand(
                    AutoBuilder.buildAuto("Source 5-4 5Y4N"),
                    new ConditionalCommand(
                        AutoBuilder.buildAuto("Source 5-4 5N4Y"),
                        AutoBuilder.buildAuto("Source 5-4 5N4N"),
                        () ->
                            !RobotContainer.POOP_MONITOR.hasPooped()
                                && RobotContainer.SHOOTER.hasNote()),
                    () ->
                        RobotContainer.POOP_MONITOR.hasPooped()
                            && !RobotContainer.SHOOTER.hasNote()),
                () -> RobotContainer.POOP_MONITOR.hasPooped() && RobotContainer.SHOOTER.hasNote()),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> RobotContainer.SHOOTER.hasNote()),
                AutoBuilder.buildAuto("Source 5-4 Preload"))),
        // new IntakeNoteSequenceAuto(
        //     RobotContainer.SHOOTER,
        //     RobotContainer.get().INTAKE,
        //     RobotContainer.get().ELEVATOR))),
        // .andThen(Util.PathFindToAutoSourceShot())
        // .andThen(new AutoAimAndShoot(RobotContainer.DRIVETRAIN, RobotContainer.SHOOTER))),
        new AutoAimLockWrist(RobotContainer.WRIST, RobotContainer::getAutoState),
        new AutoIdleShooter(RobotContainer.SHOOTER, RobotContainer::getAutoState));
  }
}
