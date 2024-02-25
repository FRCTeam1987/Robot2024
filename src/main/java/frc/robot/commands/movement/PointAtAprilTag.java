// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.movement;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

public class PointAtAprilTag extends Command {
  private LimelightHelpers limelight;

  private double kP = 0.08; // TODO: changeme please :)
  private double acceptableError = 1.0; // Degrees within acceptance
  private String limeLightName;
  private DoubleSupplier velocityXSupplier = () -> 0.0;
  private DoubleSupplier velocityYSupplier = () -> 0.0;
  private Drivetrain drivetrain;
  private SwerveRequest.ApplyChassisSpeeds swerveRequest = new SwerveRequest.ApplyChassisSpeeds();

  public PointAtAprilTag(Drivetrain drivetrain, LimelightHelpers limelight, String limeLightName) {
    this(drivetrain, limelight, limeLightName, () -> 0.0, () -> 0.0);
  }

  public PointAtAprilTag(
      Drivetrain drivetrain,
      LimelightHelpers limelight,
      String limeLightName,
      DoubleSupplier velocityXSupplier,
      DoubleSupplier velocityYSupplier) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.limeLightName = limeLightName;
    this.velocityXSupplier = velocityXSupplier;
    this.velocityYSupplier = velocityYSupplier;
  }

  @Override
  public void initialize() {
    System.out.println("Starting rotation to april tag");
  }

  @Override
  public void execute() {

    System.out.println("Starting Execute");
    if (!LimelightHelpers.getTV(limeLightName)) {
      System.out.println("NO TARGET FOUND");
      return;
    }

    double xOffset = LimelightHelpers.getTX(limeLightName);

    double rotationRate = kP * xOffset;
    System.out.println(rotationRate);

    if (Math.abs(xOffset) < acceptableError) {
      rotationRate = 0;
    }

    swerveRequest =
        swerveRequest.withSpeeds(
            new ChassisSpeeds(
                velocityXSupplier.getAsDouble(), velocityYSupplier.getAsDouble(), -rotationRate));

    // Apply the request to the drivetrain
    drivetrain.setControl(swerveRequest);
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
    drivetrain.setControl(swerveRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));

    if (interrupted) {}
  }
}
