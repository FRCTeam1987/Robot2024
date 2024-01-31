package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  private SwerveDriveKinematics kinematics;

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  // Returns the current robot pose as a Pose2d
  public Pose2d getPose() {
    return getState().Pose; // Assuming getState() returns SwerveDriveState with a Pose property
  }

  // Resets the robot's odometry to the given pose
  public void resetPose(Pose2d pose) {
    tareEverything(); // Assuming tareEverything() resets the odometry
    seedFieldRelative(
        pose); // Assuming seedFieldRelative(Pose2d) sets the odometry to a specific pose
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    // Get the states of each swerve module
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < states.length; i++) {
      // Here you would retrieve the actual wheel speed and azimuth angle
      // from each swerve module and create a SwerveModuleState.
      // This typically involves reading from the encoders and other sensors
      // on the SDS MK4i swerve modules.
      double wheelSpeed = getModule(i).getDriveMotor().getVelocity().getValueAsDouble();
      Rotation2d wheelAngle =
          Rotation2d.fromDegrees(
              getModule(i).getCANcoder().getAbsolutePosition().getValueAsDouble());
      states[i] = new SwerveModuleState(wheelSpeed, wheelAngle);
    }
    // Use the kinematics to convert swerve module states to chassis speeds
    ChassisSpeeds robotSpeeds =
        kinematics.toChassisSpeeds(states[0], states[1], states[2], states[3]);

    return robotSpeeds;
  }

  /**
   * Drives the robot using the given robot-relative speeds.
   *
   * @param speeds The robot's desired chassis speeds (vx, vy, omega).
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Create an ApplyChassisSpeeds request with the desired speeds
    SwerveRequest.ApplyChassisSpeeds driveRequest =
        new SwerveRequest.ApplyChassisSpeeds()
            .withSpeeds(speeds)
            .withCenterOfRotation(
                new Translation2d(0, 0)); // Assuming center of rotation is the robot's center

    // Apply the request to the swerve drivetrain
    SwerveControlRequestParameters parameters = new SwerveControlRequestParameters();
    parameters.kinematics = this.kinematics; // Assuming you have a kinematics object
    parameters.currentPose = this.getPose(); // Assuming you have a method to get the current pose
    parameters.updatePeriod = 0.02; // Assuming a 20ms update period
    parameters.swervePositions =
        new Translation2d[] {
          // Assuming you have a way to get the positions of the swerve modules
        };

    // Assuming you have a method to get the current swerve modules
    driveRequest.apply(parameters);
  }
}
