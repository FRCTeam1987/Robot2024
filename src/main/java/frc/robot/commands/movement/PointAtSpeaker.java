// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.movement;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class PointAtSpeaker extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.MaxSpeed * 0.1)
          .withRotationalDeadband(Constants.MaxAngularRate * 0.05) // Add a 5% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  private DoubleSupplier velocityXSupplier = () -> 0.0; // getAllianceLob
  private DoubleSupplier velocityYSupplier = () -> 0.0;
  private DoubleSupplier rotationSupplier = () -> 0.0;
  private double desiredRotation;
  private final PIDController THETA_CONTROLLER;
  private BooleanSupplier shouldLob = () -> false;
  double rotationRate = 0;

  public PointAtSpeaker(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier velocityXSupplier,
      DoubleSupplier velocityYSupplier,
      DoubleSupplier rotationSupplier) {
    this(drivetrain, velocityXSupplier, velocityYSupplier, rotationSupplier, () -> false);
  }

  public PointAtSpeaker(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier velocityXSupplier,
      DoubleSupplier velocityYSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier shouldLob) {
    this.drivetrain = drivetrain;
    this.velocityXSupplier = velocityXSupplier;
    this.velocityYSupplier = velocityYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.shouldLob = shouldLob;
    THETA_CONTROLLER = new PIDController(0.15, 0.0, 0.0); // (0.183, 0.1, 0.0013)
    THETA_CONTROLLER.enableContinuousInput(-180.0, 180.0);
    THETA_CONTROLLER.setTolerance(0.01, 0.01);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  public double getRotationRate() {
    return rotationRate;
  }

  @Override
  public void execute() {
    //Pose2d current = drivetrain.getPose();
    Pose2d pose = drivetrain.getPose();
    desiredRotation = shouldLob.getAsBoolean()
      ? Util.getRotationToAllianceLob(pose).getDegrees()
      : Util.getRotationToAllianceSpeaker(pose).getDegrees();
    double rotationRate = THETA_CONTROLLER.calculate(
      pose.getRotation().getDegrees(),
      desiredRotation
    );
    System.out.println("Current Angle: " + drivetrain.getPose().getRotation().getDegrees() + " | Desired Angle: " + desiredRotation);
    System.out.println("Rotation Rate: " + rotationRate);
    drivetrain.setControl(
        drive
            .withVelocityX(
                Util.squareValue(velocityXSupplier.getAsDouble())
                    * DriveConstants.kSpeedAt12VoltsMps) // Drive forward with
            // negative Y (forward)
            .withVelocityY(
                Util.squareValue(velocityYSupplier.getAsDouble())
                    * DriveConstants.kSpeedAt12VoltsMps) // Drive left with negative X (left)
            .withRotationalRate(rotationRate) // Drive counterclockwise with negative X (left)
        );
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // System.out.println("Command Finished!");
    FieldCentric driveRequest =
        drive
            .withVelocityX(0.0) // Drive forward with
            // negative Y (forward)
            .withVelocityY(0.0) // Drive left with negative X (left)
            .withRotationalRate(0.0); // Drive counterclockwise with negative X (left)

    drivetrain.setControl(driveRequest);
    if (interrupted) {}
  }
}
