// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.movement;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToNote2 extends PIDCommand {

  private static final SwerveRequest.ApplyChassisSpeeds swerveRequest =
      new SwerveRequest.ApplyChassisSpeeds();
  private static final Debouncer canSeeNoteDebouncer = new Debouncer(0.6, DebounceType.kFalling);

  /** Creates a new DriveToNote2. */
  public DriveToNote2(
      final CommandSwerveDrivetrain drivetrain, final Vision vision, final double initialVelocity) {
    // FIXME probably mixing degrees and radians
    super(
        // The controller that the command will use
        new PIDController(0.2, 0.0, 0.0),
        // This should return the measurement
        () -> vision.getYawVal(),
        // This should return the setpoint (can also be a constant)
        () -> 0.0,
        // This uses the output
        output -> {
          drivetrain.setControl(
              swerveRequest.withSpeeds(new ChassisSpeeds(canSeeNoteDebouncer.calculate(vision.hasTargets()) ? initialVelocity : 0.0, 0, output)));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(0.25, 0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Assume parallel deadline command stops this command.
  }
}
