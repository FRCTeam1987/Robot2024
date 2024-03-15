// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.movement;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.util.Util;

public class FastPoint extends Command {
  private final Vision photonvision;
  private final CommandSwerveDrivetrain drivetrain;
  private final PIDController thetaController;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.MaxSpeed * 0.1)
          .withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  private double initialYaw = 0.0;
  private double initalRotation = 0.0;

  public FastPoint(CommandSwerveDrivetrain drivetrain, Vision photonvision) {
    this.drivetrain = drivetrain;
    this.photonvision = photonvision;
    thetaController = new PIDController(0.20, 0.0, 0.0);
    thetaController.setTolerance(0.5, 0.25);

    thetaController.enableContinuousInput(-180, 180);
  }

  @Override
  public void initialize() {

    initialYaw = photonvision.getYawVal();
    initalRotation = drivetrain.getPose().getRotation().getDegrees();

    thetaController.setSetpoint(initalRotation - initialYaw);
    // System.out.println("Starting rotation to april tag");
  }

  @Override
  public void execute() {
    double rotationalVelocity =
        thetaController.calculate(
                drivetrain.getPose().getRotation().getDegrees(), thetaController.getSetpoint())
            * 6;
    FieldCentric driveRequest =
        drive
            .withVelocityX(0.0) // Drive forward with
            // negative Y (forward)
            .withVelocityY(0.0) // Drive left with negative X (left)
            .withRotationalRate(
                rotationalVelocity); // Drive counterclockwise with negative X (left)
    System.out.println(rotationalVelocity);
    drivetrain.setControl(driveRequest);
  }

  @Override
  public boolean isFinished() {
    // return false;
    return Util.isWithinTolerance(photonvision.getYawVal(), 0.0, 0.02);
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
