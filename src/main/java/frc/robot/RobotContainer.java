// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.control.AimLockWrist;
import frc.robot.commands.control.Climb;
import frc.robot.commands.control.GoHome;
import frc.robot.commands.control.IdleShooter;
import frc.robot.commands.control.IntakeNoteSequence;
import frc.robot.commands.control.PoopNote;
import frc.robot.commands.control.PrepareShootAmp;
import frc.robot.commands.control.ReverseIntake;
import frc.robot.commands.control.ShootAmp;
import frc.robot.commands.control.ShootNote;
import frc.robot.commands.control.ShootNoteSequence;
import frc.robot.commands.control.ShootSubwoofer;
import frc.robot.commands.control.ShootSubwooferFlat;
import frc.robot.commands.control.ShootTall;
import frc.robot.commands.control.SimpleShootforAmp;
import frc.robot.commands.control.SpitNote;
import frc.robot.commands.control.StopAll;
import frc.robot.commands.movement.CollectNoteAuto;
import frc.robot.commands.movement.DriveToNote;
import frc.robot.commands.movement.DriveToNoteAuto;
import frc.robot.commands.movement.PointAtAprilTag;
import frc.robot.commands.movement.SquareUpToAprilTag;
import frc.robot.commands.movement.TeleopSwerve;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.candle.Candles;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.wrist.Wrist;
import java.util.Arrays;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class RobotContainer {
  private static RobotContainer instance;
  public final ShuffleboardTab COMMANDS_TAB = Shuffleboard.getTab("COMMANDS");
  public final ShuffleboardTab MATCH_TAB = Shuffleboard.getTab("MATCH");
  public final ShuffleboardTab PHOTON_TAB = Shuffleboard.getTab("PHOTON");
  public final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("SHOOTER");

  private final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(0);
  private final CommandXboxController CO_DRIVER_CONTROLLER = new CommandXboxController(1);

  public final PVision VISION_SPEAKER =
      new PVision(
          VisionConstants.CAMERA_NAME_SPEAKER,
          VisionConstants.TAG_LAYOUT,
          VisionConstants.ROBOT_TO_CAM_SPEAKER);
//   public final PVision VISION_AMP =
//       new PVision(
//           VisionConstants.CAMERA_NAME_AMP,
//           VisionConstants.TAG_LAYOUT,
//           VisionConstants.ROBOT_TO_CAM_AMP);

  public final Vision INTAKE_PHOTON = new Vision("Arducam_OV9782_USB_Camera", 0.65176, 60);
  public final Vision SPEAKER_PHOTON =
      new Vision("Arducam_OV2311_USB_Camera_1", 0.3, 40, Arrays.asList(4, 7));
  public final Vision AMP_PHOTON =
      new Vision("Arducam_OV2311_USB_Camera", 0.35, 40, Arrays.asList(5, 6));

  public final Drivetrain DRIVETRAIN = DriveConstants.DriveTrain; // My drivetrain
  public final Candles CANDLES = new Candles(Constants.LEFT_CANDLE, Constants.RIGHT_CANDLE);
  public final Intake INTAKE = new Intake(Constants.INTAKE_TOP_ID, Constants.INTAKE_BOTTOM_ID);
  public final Shooter SHOOTER =
      new Shooter(
          Constants.SHOOTER_LEADER_ID,
          Constants.SHOOTER_FOLLOWER_ID,
          Constants.SHOOTER_FEEDER_ID,
          SPEAKER_PHOTON);
  public final Wrist WRIST = new Wrist(Constants.WRIST_ID);
  public final Elevator ELEVATOR =
      new Elevator(Constants.ELEVATOR_LEADER_ID, Constants.ELEVATOR_FOLLOWER_ID);

  private final Telemetry logger = new Telemetry(Constants.MaxSpeed);
  private final SendableChooser<Command> AUTO_CHOOSER = new SendableChooser<>();

  private boolean isAmpPrimed = false;
  private boolean isClimbPrimed = false;
  private static boolean isAmpPrepped = false;

  private void configureDriverController() {
    DRIVER_CONTROLLER
        .y()
        .onTrue(
            new ConditionalCommand(
                new SimpleShootforAmp(SHOOTER, ELEVATOR, WRIST)
                    .andThen(new WaitCommand(0.2))
                    .andThen(new InstantCommand(() -> isAmpPrimed = false)),
                new PrepareShootAmp(ELEVATOR, WRIST)
                    .andThen(new WaitCommand(0.2))
                    .andThen(new InstantCommand(() -> isAmpPrimed = true)),
                () -> isAmpPrimed));

    DRIVER_CONTROLLER
        .back()
        .onTrue(
            new InstantCommand(
                () -> {
                  DRIVETRAIN.resetPose(new Pose2d());
                  System.out.println(DRIVETRAIN.getPose().getRotation());
                },
                DRIVETRAIN));
    DRIVER_CONTROLLER.start().onTrue(new GoHome(ELEVATOR, WRIST, SHOOTER, INTAKE));
    DRIVER_CONTROLLER.x().onTrue(new PoopNote(SHOOTER, 500));
    DRIVER_CONTROLLER
        .leftBumper()
        .onTrue(
            new IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR)
                .andThen(
                    new InstantCommand(
                            () -> DRIVER_CONTROLLER.getHID().setRumble(RumbleType.kBothRumble, 1.0))
                        .andThen(new WaitCommand(0.7))
                        .andThen(
                            new InstantCommand(
                                () -> {
                                  DRIVER_CONTROLLER.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                                  CANDLES.setColor(0, 128, 0);
                                }))));

    DRIVER_CONTROLLER
        .leftTrigger()
        .whileTrue(
            new PointAtAprilTag(
                DRIVETRAIN,
                SPEAKER_PHOTON,
                () -> (DRIVER_CONTROLLER.getLeftX() * Constants.MaxSpeed),
                () -> (DRIVER_CONTROLLER.getLeftY() * Constants.MaxSpeed),
                () -> (DRIVER_CONTROLLER.getRightX() * Constants.MaxSpeed)));

    DRIVER_CONTROLLER
        .rightBumper()
        .onTrue(new ShootNote(SHOOTER, ELEVATOR, ShooterConstants.SHOOTER_RPM));
    DRIVER_CONTROLLER.a().onTrue(new ShootAmp(SHOOTER, ELEVATOR, WRIST));
    DRIVER_CONTROLLER.rightTrigger().onTrue(new ShootSubwoofer(ELEVATOR, WRIST, SHOOTER));
  }

  private void configureCoDriverController() {
    CO_DRIVER_CONTROLLER.start().onTrue(new StopAll(WRIST, SHOOTER, INTAKE, ELEVATOR));
    CO_DRIVER_CONTROLLER.rightBumper().onTrue(new PoopNote(SHOOTER, 2500));

    CO_DRIVER_CONTROLLER
        .y()
        .onTrue(
            new ConditionalCommand(
                new Climb(ELEVATOR, WRIST, SHOOTER),
                new InstantCommand(
                        () -> ELEVATOR.setLengthInches(Constants.ELEVATOR_TRAP_HEIGHT), ELEVATOR)
                    .andThen(() -> isClimbPrimed = true),
                () -> isClimbPrimed));

    CO_DRIVER_CONTROLLER.x().onTrue(new ReverseIntake(SHOOTER, INTAKE, WRIST, ELEVATOR));
    CO_DRIVER_CONTROLLER.leftTrigger().onTrue(new ShootTall(ELEVATOR, WRIST, SHOOTER));
    CO_DRIVER_CONTROLLER
        .b()
        .onTrue(
            new InstantCommand(
                    () -> {
                      SHOOTER.setFeederVoltage(ShooterConstants.FEEDER_FEEDFWD_VOLTS);
                      INTAKE.setVolts(Constants.INTAKE_COLLECT_VOLTS);
                    })
                .andThen(new WaitCommand(0.1))
                .andThen(
                    new InstantCommand(
                        () -> {
                          SHOOTER.stopFeeder();
                          INTAKE.stopTop();
                          INTAKE.stopCollecting();
                        })));
    CO_DRIVER_CONTROLLER
        .leftBumper()
        .whileTrue(
            new PointAtAprilTag(
                DRIVETRAIN,
                SPEAKER_PHOTON,
                () -> (DRIVER_CONTROLLER.getLeftX() * Constants.MaxSpeed),
                () -> (DRIVER_CONTROLLER.getLeftY() * Constants.MaxSpeed),
                () -> (DRIVER_CONTROLLER.getRightX() * Constants.MaxSpeed)));

    CO_DRIVER_CONTROLLER.rightTrigger().onTrue(new ShootSubwooferFlat(ELEVATOR, WRIST, SHOOTER));
  }

  private void configureDrivetrain() {

    DRIVETRAIN.setDefaultCommand(
        new TeleopSwerve(
            DRIVETRAIN,
            () -> -DRIVER_CONTROLLER.getLeftY(),
            () -> -DRIVER_CONTROLLER.getLeftX(),
            DRIVER_CONTROLLER::getRightX,
            () -> 1.0,
            () -> DRIVER_CONTROLLER.getHID().getPOV(),
            () -> false));

    if (Utils.isSimulation()) {
      DRIVETRAIN.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    DRIVETRAIN.registerTelemetry(logger::telemeterize);
  }

  public void configureShuffleboard() {
    SHOOTER_TAB.add("Coast Wrist", new InstantCommand(WRIST::coast).ignoringDisable(true));
    SHOOTER_TAB.add("Brake Wrist", new InstantCommand(WRIST::brake).ignoringDisable(true));
    COMMANDS_TAB.addNumber("Speaker Pitch", SPEAKER_PHOTON::getPitchVal);
    COMMANDS_TAB.add(
        "Intake Auto",
        new ParallelCommandGroup(
            new DriveToNoteAuto(DRIVETRAIN, AMP_PHOTON, SHOOTER, INTAKE, WRIST, ELEVATOR),
            new IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR)));
    COMMANDS_TAB.add(
        "Force Zero All",
        new InstantCommand(
                () -> {
                  ELEVATOR.setZero();
                  WRIST.setZero();
                })
            .ignoringDisable(true));
    MATCH_TAB.addBoolean("Center LB", SHOOTER::isCenterBroken).withPosition(0, 1);
    MATCH_TAB.addBoolean("Rear LB", SHOOTER::isRearBroken).withPosition(0, 2);
    MATCH_TAB
        .add("Reverse Intake", new ReverseIntake(SHOOTER, INTAKE, WRIST, ELEVATOR))
        .withPosition(1, 0);
    MATCH_TAB
        .add("Wrist +1 deg", new InstantCommand(() -> WRIST.incrementWrist(1)))
        .withPosition(1, 1);
    MATCH_TAB
        .add("Wrist -1 deg", new InstantCommand(() -> WRIST.incrementWrist(-1)))
        .withPosition(2, 1);
    MATCH_TAB.addDouble("Wrist Incr", WRIST::getIncrementValue).withPosition(3, 1);
    MATCH_TAB
        .add("Elev +1 in", new InstantCommand(() -> ELEVATOR.incrementElevator(1)))
        .withPosition(1, 2);
    MATCH_TAB
        .add("Elev -1 in.", new InstantCommand(() -> ELEVATOR.incrementElevator(-1)))
        .withPosition(2, 2);
    MATCH_TAB.addDouble("Elev Incr", ELEVATOR::getIncrementValue).withPosition(3, 2);
    MATCH_TAB
        .add(
            "InstaSuck",
            new InstantCommand(
                    () -> {
                      SHOOTER.setFeederVoltage(ShooterConstants.FEEDER_FEEDFWD_VOLTS);
                      INTAKE.setVolts(Constants.INTAKE_COLLECT_VOLTS);
                    })
                .andThen(new WaitCommand(0.1))
                .andThen(
                    new InstantCommand(
                        () -> {
                          SHOOTER.stopFeeder();
                          INTAKE.stopTop();
                          INTAKE.stopCollecting();
                        })))
        .withPosition(0, 3);
    MATCH_TAB.add("Prepare Shoot Amp", new PrepareShootAmp(ELEVATOR, WRIST)).withPosition(1, 5);
    MATCH_TAB.add("Shoot Amp", new ShootAmp(SHOOTER, ELEVATOR, WRIST)).withPosition(1, 6);
    MATCH_TAB
        .add(
            "Prep Climb",
            new InstantCommand(() -> ELEVATOR.setLengthInches(Constants.ELEVATOR_TRAP_HEIGHT)))
        .withPosition(2, 5);
    // MATCH_TAB.add("Climb", new Climb(ELEVATOR, CLIMBER, WRIST, SHOOTER)).withPosition(2, 6);
    MATCH_TAB.add("Poop Long", new PoopNote(SHOOTER, 3500));
    MATCH_TAB.add(
        "Reset Amp",
        new InstantCommand(
            () -> {
              isAmpPrepped = false;
              isAmpPrimed = false;
            }));

    COMMANDS_TAB.add("Shoot Amp", new ShootAmp(SHOOTER, ELEVATOR, WRIST));
    COMMANDS_TAB.add("Shoot Subwoofer", new ShootSubwoofer(ELEVATOR, WRIST, SHOOTER));
    COMMANDS_TAB.add("Shoot Tall", new ShootTall(ELEVATOR, WRIST, SHOOTER));
    COMMANDS_TAB.add(
        "Subwoofer Shot",
        new ShootNoteSequence(SHOOTER, WRIST, ELEVATOR, ShooterConstants.SHOOTER_RPM, 52, 2));
    COMMANDS_TAB.add("Poop Note", new PoopNote(SHOOTER, 500));
    COMMANDS_TAB.addDouble(
        "Distance of Last Shot",
        () -> SHOOTER.ShooterCameraDistanceToTarget(Constants.SPEAKER_APRILTAG_HEIGHT));
    COMMANDS_TAB.add(
        "IntakeNote",
        new frc.robot.commands.control.IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR));
    COMMANDS_TAB.add("Set Wrist as at Home", new InstantCommand(WRIST::zeroSensor));
    COMMANDS_TAB.add(
        "ShootNote", new frc.robot.commands.control.ShootNoteSequence(SHOOTER, WRIST, 6000, 0));
    COMMANDS_TAB.add("SpinUpShooter", new InstantCommand(() -> SHOOTER.setRPMShoot(1800)));
    COMMANDS_TAB.add("StopShooter", new InstantCommand(() -> SHOOTER.setRPMShoot(0)));
    PHOTON_TAB.add("Rotate to AprilTag", new PointAtAprilTag(DRIVETRAIN, SPEAKER_PHOTON));
    PHOTON_TAB.addDouble("Current Heading", () -> DRIVETRAIN.getPose().getRotation().getDegrees());
    PHOTON_TAB.addDouble("Current poseX", () -> DRIVETRAIN.getPose().getX());

    COMMANDS_TAB.add(
        "Driving Rotate to AprilTag",
        new PointAtAprilTag(
            DRIVETRAIN,
            SPEAKER_PHOTON,
            () -> (DRIVER_CONTROLLER.getLeftY() * Constants.MaxSpeed),
            () -> (DRIVER_CONTROLLER.getLeftX() * Constants.MaxSpeed),
            () -> (DRIVER_CONTROLLER.getRightX() * Constants.MaxSpeed)));
    PHOTON_TAB.add(
        "Square Up AprilTag",
        new SquareUpToAprilTag(DRIVETRAIN, SPEAKER_PHOTON, Constants.SPEAKER_APRILTAG_HEIGHT, 3));
    PHOTON_TAB.add(
        "Climb Test",
        new SquareUpToAprilTag(DRIVETRAIN, SPEAKER_PHOTON, Constants.TRAP_APRILTAG_HEIGHT, 3)
        // .andThen(
        //     new InstantCommand(
        //             () ->
        //                 drivetrain.resetPose(
        //                     new Pose2d(2, 7, new Rotation2d(0)))) // Starting position of path
        //         .andThen(drivetrain.followPathCommand("Taxi")))
        );
    // LIMELIGHT_TAB.add(
    //     "Taxi path",
    //     new InstantCommand(
    //             () ->
    //                 DRIVETRAIN.seedFieldRelative(
    //                     new Pose2d(2, 7, new Rotation2d(0)))) // Starting position of path
    //         .andThen(DRIVETRAIN.("Taxi")));

    PHOTON_TAB.add(
        "Drive To Note",
        new DriveToNote(DRIVETRAIN, () -> -DRIVER_CONTROLLER.getLeftY(), INTAKE_PHOTON));
    PHOTON_TAB.addNumber(
        "Distance from Speaker",
        () ->
            Vision.calculateDistanceToTarget(
                SPEAKER_PHOTON.getPitchVal(),
                SPEAKER_PHOTON.getCameraHeight(),
                Constants.SPEAKER_APRILTAG_HEIGHT,
                SPEAKER_PHOTON.getCameraDegrees()));
    PHOTON_TAB.add(
        "Drive To Note Auto",
        new DriveToNoteAuto(DRIVETRAIN, INTAKE_PHOTON, SHOOTER, INTAKE, WRIST, ELEVATOR));
    PHOTON_TAB.add(
        "Collect Note Auto",
        new CollectNoteAuto(DRIVETRAIN, SHOOTER, INTAKE, WRIST, ELEVATOR, INTAKE_PHOTON));
    PHOTON_TAB.addNumber("Pose x", () -> DRIVETRAIN.getPose().getX());
    PHOTON_TAB.addNumber("Pose y", () -> DRIVETRAIN.getPose().getY());
    PHOTON_TAB.addNumber("Pose Theta", () -> DRIVETRAIN.getPose().getRotation().getDegrees());

    SHOOTER_TAB.add("Spit Note", new SpitNote(SHOOTER));

    addAuto("ampa-full");
    addAuto("sourcea-fullshoot");
    addAuto("sourcea");
    addAuto("amp_close");
    addAuto("amp_subwoofer");
    addAuto("amp_subwoofer_reversal");
    AUTO_CHOOSER.addOption("Do Nothing", new InstantCommand());
    MATCH_TAB.add("Auto", AUTO_CHOOSER);
  }

  public void configureDefaultCommands() {
    WRIST.setDefaultCommand(new AimLockWrist(WRIST, SHOOTER, ELEVATOR, SPEAKER_PHOTON));
    SHOOTER.setDefaultCommand(new IdleShooter(SHOOTER));
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand(
        "ShootNote", new ShootNoteSequence(SHOOTER, WRIST, ShooterConstants.SHOOTER_RPM, 40));
    NamedCommands.registerCommand(
        "ShootNoteAimbot",
        new ShootNoteSequence(
            SHOOTER, WRIST, ShooterConstants.SHOOTER_RPM, DRIVETRAIN, SPEAKER_PHOTON));
    NamedCommands.registerCommand(
        "SpinUpShooter", new InstantCommand(() -> SHOOTER.setRPMShoot(5200)));
    NamedCommands.registerCommand(
        "ShootNoteSubFar",
        new ShootNoteSequence(SHOOTER, WRIST, ELEVATOR, ShooterConstants.SHOOTER_RPM, 36, 10));
    NamedCommands.registerCommand(
        "IntakeNote", new IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR));
    NamedCommands.registerCommand(
        "PoopPrep",
        new InstantCommand(
            () -> {
              SHOOTER.setRPMShootNoSpin(650);
              INTAKE.setVolts(-8.0);
              WRIST.setDegrees(15);
            },
            SHOOTER,
            WRIST,
            INTAKE));
    NamedCommands.registerCommand(
        "PoopPrep2500",
        new InstantCommand(
            () -> {
              SHOOTER.setRPMShootNoSpin(2500);
              WRIST.setDegrees(15);
            },
            SHOOTER,
            WRIST,
            INTAKE));
    NamedCommands.registerCommand(
        "PoopStart", new InstantCommand(() -> SHOOTER.setFeederVoltage(7.0)));
    NamedCommands.registerCommand("PoopPause", new InstantCommand(SHOOTER::stopShooter, SHOOTER));
    NamedCommands.registerCommand(
        "PoopStop",
        new InstantCommand(
            () -> {
              SHOOTER.stopShooter();
              SHOOTER.stopFeeder();
              INTAKE.stopCollecting();
            },
            SHOOTER,
            INTAKE));
    NamedCommands.registerCommand("ShootSubwoofer", new ShootSubwoofer(ELEVATOR, WRIST, SHOOTER));
    NamedCommands.registerCommand("ShootAmp", new ShootAmp(SHOOTER, ELEVATOR, WRIST));
    NamedCommands.registerCommand(
        "DriveToNoteAuto",
        new ParallelDeadlineGroup(
            new IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR),
            new DriveToNoteAuto(DRIVETRAIN, AMP_PHOTON, SHOOTER, INTAKE, WRIST, ELEVATOR)));
  }

  public void addAuto(String autoName) {
    AUTO_CHOOSER.addOption(autoName, new PathPlannerAuto(autoName));
  }

  public RobotContainer() {
    instance = this;
    configureNamedCommands();
    configureShuffleboard();
    configureDrivetrain();
    configureDriverController();
    configureCoDriverController();
    configureDefaultCommands();
  }

  public static RobotContainer get() {
    return instance;
  }

  public Command getAutonomousCommand() {
    return AUTO_CHOOSER.getSelected();
  }

  public static boolean isAmpPrepped() {
    return isAmpPrepped;
  }

  public static void setAmpPrepped(final boolean isPrepped) {
    isAmpPrepped = isPrepped;
  }

  public void visionPeriodic() {
    Optional<EstimatedRobotPose> speakerEstimate = VISION_SPEAKER.getEstimatedGlobalPose();
    speakerEstimate.ifPresent(
        estimate -> {
          Pose2d estimatedPose = estimate.estimatedPose.toPose2d();
          Matrix<N3, N1> estStdDevs = VISION_SPEAKER.getEstimationStdDevs(estimatedPose);
          DRIVETRAIN.addVisionMeasurement(estimatedPose, estimate.timestampSeconds, estStdDevs);
        });
    // Optional<EstimatedRobotPose> ampEstimate = VISION_AMP.getEstimatedGlobalPose();
    // ampEstimate.ifPresent(
    //     estimate -> {
    //       Pose2d estimatedPose = estimate.estimatedPose.toPose2d();
    //       Matrix<N3, N1> estStdDevs = VISION_AMP.getEstimationStdDevs(estimatedPose);
    //       DRIVETRAIN.addVisionMeasurement(estimatedPose, estimate.timestampSeconds, estStdDevs);
    //     });
  }

  public static final double DEADBAND = 0.05;

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value input value
   * @return square of the value
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
