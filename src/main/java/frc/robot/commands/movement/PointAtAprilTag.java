// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.movement;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class PointAtAprilTag extends Command {
  private final Vision photonvision;
  private final CommandSwerveDrivetrain drivetrain;
  private final SlewRateLimiter translationXSlewRate =
      new SlewRateLimiter(Constants.translationXSlewRate);
  private final SlewRateLimiter translationYSlewRate =
      new SlewRateLimiter(Constants.translationYSlewRate);
  private final SlewRateLimiter rotationSlewRate = new SlewRateLimiter(1);
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.MaxSpeed * 0.1)
          .withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  private DoubleSupplier velocityXSupplier = () -> 0.0;
  private DoubleSupplier velocityYSupplier = () -> 0.0;
  private DoubleSupplier rotationSupplier = () -> 0.0;
  private DoubleSupplier rotationSupplier2 = () -> 0.0;

  double rotationRate = 0;

  public PointAtAprilTag( // USE FAST POINT INSTEAD. DO NOT USE COMMAND IT IS UNRELIABLE
      CommandSwerveDrivetrain drivetrain,
      Vision photonvision,
      DoubleSupplier velocityXSupplier,
      DoubleSupplier velocityYSupplier,
      DoubleSupplier rotationSupplier) {
    this.drivetrain = drivetrain;
    this.photonvision = photonvision;
    this.velocityXSupplier = velocityXSupplier;
    this.velocityYSupplier = velocityYSupplier;
    this.rotationSupplier = rotationSupplier;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // System.out.println("Starting rotation to april tag");

    // FieldCentric driveRequest =
    //     drive
    //         .withVelocityX(
    //             translationYSlewRate.calculate(
    //                 -velocityYSupplier.getAsDouble())) // Drive forward with
    //         // negative Y (forward)
    //         .withVelocityY(
    //             translationXSlewRate.calculate(
    //                 -velocityXSupplier.getAsDouble())) // Drive left with negative X (left)
    //         .withRotationalRate(
    //             -this.getRotationRate()); // Drive counterclockwise with negative X (left)

    // drivetrain.setControl(driveRequest);
  }

  public double getRotationRate() {
    return rotationRate;
  }

  @Override
  public void execute() {

    // System.out.println("Starting Execute");

    double xOffset = photonvision.getYawVal();

    // TODO: changeme please :)
    double kP = 0.15;

    if (xOffset > 1) {
      rotationRate = kP * xOffset;
    } else {
      rotationRate = xOffset * 0.09;
    }

    System.out.println("Attempted RotationRate: " + rotationRate);
    // rotationRate = Math.copySign(MathUtil.clamp(Math.abs(rotationRate), 0, 2.75), rotationRate);

    // double rotationRate = kP * xOffset;

    // Degrees within acceptance

    double acceptableError = 0.15;
    if (Math.abs(rotationRate) < acceptableError) {
      rotationRate = 0;
    }

    System.out.println(rotationRate);

    if (!photonvision.hasTargets()) {
      rotationRate = rotationSupplier.getAsDouble();
      DriverStation.reportWarning("No Tag found in PointAtAprilTag", false);
    }
    drivetrain.setControl(
        drive
            .withVelocityX(
                translationYSlewRate.calculate(
                    -velocityYSupplier.getAsDouble())) // Drive forward with
            // negative Y (forward)
            .withVelocityY(
                translationXSlewRate.calculate(
                    -velocityXSupplier.getAsDouble())) // Drive left with negative X (left)
            .withRotationalRate(-rotationRate) // Drive counterclockwise with negative X (left)
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
