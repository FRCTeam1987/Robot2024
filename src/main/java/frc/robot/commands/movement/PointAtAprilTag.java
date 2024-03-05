// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.movement;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class PointAtAprilTag extends Command {
  private Vision photonvision;

  private double kP = 0.14; // TODO: changeme please :)
  private double acceptableError = 1.0; // Degrees within acceptance
  private String photonName;
  private DoubleSupplier velocityXSupplier = () -> 0.0;
  private DoubleSupplier velocityYSupplier = () -> 0.0;
  private DoubleSupplier rotationSupplier = () -> 0.0;
  private Drivetrain drivetrain;
  private final SlewRateLimiter translationXSlewRate =
      new SlewRateLimiter(Constants.translationXSlewRate);
  private final SlewRateLimiter translationYSlewRate =
      new SlewRateLimiter(Constants.translationYSlewRate);
  private final SlewRateLimiter rotationSlewRate = new SlewRateLimiter(Constants.rotationSlewRate);
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.MaxSpeed * 0.1)
          .withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  public PointAtAprilTag(Drivetrain drivetrain, Vision photonvision) {
    this(drivetrain, photonvision, () -> 0.0, () -> 0.0, () -> 0.0);
  }

  public PointAtAprilTag(
      Drivetrain drivetrain,
      Vision photonvision,
      DoubleSupplier velocityXSupplier,
      DoubleSupplier velocityYSupplier,
      DoubleSupplier rotationSupplier) {
    this.drivetrain = drivetrain;
    this.photonvision = photonvision;
    this.photonName = photonName;
    this.velocityXSupplier = velocityXSupplier;
    this.velocityYSupplier = velocityYSupplier;
    this.rotationSupplier = rotationSupplier;
  }

  @Override
  public void initialize() {
    System.out.println("Starting rotation to april tag");
  }

  @Override
  public void execute() {

    System.out.println("Starting Execute");

    double xOffset = photonvision.getYawVal();

    double rotationRate = kP * xOffset;
    System.out.println(rotationRate);

    if (Math.abs(xOffset) < acceptableError) {
      rotationRate = 0;
    }

    if (!photonvision.hasTargets()) {
      rotationRate = rotationSupplier.getAsDouble();
    }

    FieldCentric driveRequest =
        drive
            .withVelocityX(
                translationYSlewRate.calculate(
                    velocityYSupplier.getAsDouble())) // Drive forward with
            // negative Y (forward)
            .withVelocityY(
                translationXSlewRate.calculate(
                    velocityXSupplier.getAsDouble())) // Drive left with negative X (left)
            .withRotationalRate(
                rotationSlewRate.calculate(
                    -rotationRate)); // Drive counterclockwise with negative X (left)

    drivetrain.setControl(driveRequest);
  }

  @Override
  public boolean isFinished() {
    // double xOffset = limelight.getTX(limeLightName);
    // return Math.abs(xOffset) < acceptableError;
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Command Finished!");
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
