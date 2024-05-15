// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.control.AimLockWrist;
import frc.robot.commands.control.AimLockWristAuto;
import frc.robot.commands.control.Climb;
import frc.robot.commands.control.GoHome;
import frc.robot.commands.control.IdleShooter;
import frc.robot.commands.control.NewShootAmpAuto;
import frc.robot.commands.control.ReverseIntake;
import frc.robot.commands.control.ShootAmp;
import frc.robot.commands.control.ShootSubwoofer;
import frc.robot.commands.control.ShootSubwooferFirstHalf;
import frc.robot.commands.control.ShootSubwooferFlat;
import frc.robot.commands.control.ShootTall;
import frc.robot.commands.control.StopAll;
import frc.robot.commands.control.amp.FireRevAmp;
import frc.robot.commands.control.amp.PrepRevAmp;
import frc.robot.commands.control.auto.AutoAimAndShoot;
import frc.robot.commands.control.auto.AutoAimLockWrist;
import frc.robot.commands.control.auto.AutoCollectNote;
import frc.robot.commands.control.auto.AutoIdleShooter;
import frc.robot.commands.control.auto.InstantShoot;
import frc.robot.commands.control.note.IntakeNoteSequence;
import frc.robot.commands.control.note.IntakeNoteSequenceAuto;
import frc.robot.commands.control.note.LobNote;
import frc.robot.commands.control.note.PoopNote;
import frc.robot.commands.control.note.ShootNote;
import frc.robot.commands.control.note.ShootNoteAimbotFixed;
import frc.robot.commands.control.note.ShootNoteSequence;
import frc.robot.commands.control.note.SpitNote;
import frc.robot.commands.movement.CollectNoteAuto;
import frc.robot.commands.movement.DriveToNote;
import frc.robot.commands.movement.DriveToNoteAuto;
import frc.robot.commands.movement.PointAtAprilTag;
import frc.robot.commands.movement.SwerveCommand;
import frc.robot.commands.qol.AsyncRumble;
import frc.robot.commands.qol.DefaultCANdle;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.AmpSensors;
import frc.robot.subsystems.Candles;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Util;

public class RobotContainer {
  private final SendableChooser<Command> AUTO_CHOOSER = new SendableChooser<>();
  public final ShuffleboardTab COMMANDS_TAB = Shuffleboard.getTab("COMMANDS");
  public final ShuffleboardTab MATCH_TAB = Shuffleboard.getTab("MATCH");
  public final ShuffleboardTab PHOTON_TAB = Shuffleboard.getTab("PHOTON");
  public final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("SHOOTER");
  public final ShuffleboardTab PROTO_TAB = Shuffleboard.getTab("PROTO");
  public static final Vision INTAKE_PHOTON = new Vision("Arducam_OV9782_USB_Camera", 0.651830, 60);
  public final CommandSwerveDrivetrain DRIVETRAIN = DriveConstants.DriveTrain; // My drivetrain
  public final Candles CANDLES = new Candles(Constants.LEFT_CANDLE, Constants.RIGHT_CANDLE);
  public final Intake INTAKE = new Intake(Constants.INTAKE_TOP_ID, Constants.INTAKE_BOTTOM_ID);
  public final Shooter SHOOTER =
      new Shooter(
          Constants.SHOOTER_LEADER_ID, Constants.SHOOTER_FOLLOWER_ID, Constants.SHOOTER_FEEDER_ID);
  public final Wrist WRIST = new Wrist(Constants.WRIST_ID);
  public final Elevator ELEVATOR =
      new Elevator(Constants.ELEVATOR_LEADER_ID, Constants.ELEVATOR_FOLLOWER_ID);
  public final AmpSensors AMP_SENSORS = new AmpSensors();
  private final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(0);
  private final CommandXboxController CO_DRIVER_CONTROLLER = new CommandXboxController(1);
  private final SlewRateLimiter TranslationXSlewRate =
      new SlewRateLimiter(Constants.translationSlewRate);
  private final SlewRateLimiter TranslationYSlewRate =
      new SlewRateLimiter(Constants.translationSlewRate);
  // private final SlewRateLimiter RotationalSlewRate =
  //     new SlewRateLimiter(Constants.rotationSlewRate);
  public static boolean isAmpPrepped = false;
  public static boolean isAmpShot = false;
  public static boolean isClimbPrimed = false;
  public static boolean aimAtTargetAuto = false;
  public static boolean teleopShouldPointToNote = false;
  private double MaxSpeed = DriveConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = 1.5 * Math.PI;
  private boolean VisionUpdate = false;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.03)
          .withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private static RobotContainer instance;

  public static void enableTeleopPointToNote() {
    teleopShouldPointToNote = true;
  }

  public static void disableTeleopPointToNote() {
    teleopShouldPointToNote = false;
  }

  public static boolean shouldTelopPointToNote() {
    return teleopShouldPointToNote;
  }

  public RobotContainer() {
    instance = this;

    new Util();
    configureNamedCommands();
    configureShuffleboard();
    configureDrivetrain();
    configureDriverController();
    configureCoDriverController();
    configureDefaultCommands();
  }

  private void configureDriverController() {
    DRIVER_CONTROLLER.b().onTrue(new ShootSubwooferFlat(ELEVATOR, WRIST, SHOOTER));

    // DRIVER_CONTROLLER
    //     .y()
    //     .onTrue(
    //         new ConditionalCommand(
    //             new GoHome(ELEVATOR, WRIST, SHOOTER, INTAKE)
    //                 .andThen(() -> isReverseAmpPrimed = false),
    //             new PrepRevAmp(ELEVATOR, WRIST)
    //                 .andThen(new WaitCommand(0.8))
    //                 .andThen(new FireRevAmp(SHOOTER))
    //                 .andThen(new WaitCommand(0.1))
    //                 .andThen(new InstantCommand(() -> ELEVATOR.setLengthInches(4.2)))
    //                 .andThen(new InstantCommand(() -> isReverseAmpPrimed = true)),
    //             () -> isReverseAmpPrimed));

    DRIVER_CONTROLLER
        .y()
        .onTrue(
            new ConditionalCommand(
                // Compare prepped to not prepped (If prepped, try shoot. Else, prep)
                new ConditionalCommand(
                    // Compare shot to not shot (If shot, go home. If not, try shot)
                    new GoHome(ELEVATOR, WRIST, SHOOTER, INTAKE)
                        .andThen(
                            () -> {
                              isAmpPrepped = false;
                              isAmpShot = false;
                            }),
                    new ConditionalCommand(
                        // Compare working shot (If working, shoot. Else rumble.)
                        new FireRevAmp(SHOOTER)
                            .andThen(new WaitCommand(0.1))
                            .andThen(new InstantCommand(() -> ELEVATOR.setLengthInches(4.2)))
                            .andThen(new InstantCommand(() -> isAmpShot = true)),
                        new AsyncRumble(
                            DRIVER_CONTROLLER.getHID(), RumbleType.kBothRumble, 1.0, 400L),
                        () -> AMP_SENSORS.getBothSensors()),
                    () -> isAmpShot),
                new PrepRevAmp(ELEVATOR, WRIST)
                    .andThen(new InstantCommand(() -> isAmpPrepped = true)),
                () -> isAmpPrepped));
    DRIVER_CONTROLLER
        .back()
        .onTrue(
            DRIVETRAIN
                .runOnce(
                    () -> {
                      DRIVETRAIN.seedFieldRelative();
                      DRIVETRAIN.getPigeon2().reset();
                    })
                .andThen(new InstantCommand(() -> updatePoseVision(0.01, false))));
    // DRIVER_CONTROLLER.back().onTrue(new InstantCommand(() -> updatePoseVision(0.01, false)));
    DRIVER_CONTROLLER
        .start()
        .onTrue(
            new GoHome(ELEVATOR, WRIST, SHOOTER, INTAKE)
                .andThen(new InstantCommand(() -> WRIST.goHome())));
    DRIVER_CONTROLLER.x().onTrue(new PoopNote(SHOOTER, 500));
    DRIVER_CONTROLLER
        .leftBumper()
        .onTrue(
                new IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR)
                .andThen(
                    new AsyncRumble(DRIVER_CONTROLLER.getHID(), RumbleType.kBothRumble, 1.0, 700L))
            // No instantcommand wrapper?
            ).whileTrue(new InstantCommand(() -> enableTeleopPointToNote()))
            .onFalse(new InstantCommand(() -> disableTeleopPointToNote()));

    DRIVER_CONTROLLER
        .rightTrigger()
        .whileTrue(
            new PointAtAprilTag(
                DRIVETRAIN,
                () -> -TranslationXSlewRate.calculate(DRIVER_CONTROLLER.getLeftY()),
                () -> -TranslationYSlewRate.calculate(DRIVER_CONTROLLER.getLeftX()),
                () -> DRIVER_CONTROLLER.getRightX()));

    DRIVER_CONTROLLER
        .rightBumper()
        .onTrue(new ShootNote(SHOOTER, ELEVATOR, Constants.Shooter.SHOOTER_RPM));
    //DRIVER_CONTROLLER.leftTrigger(0.1).onTrue(new LobNote(SHOOTER, WRIST, ELEVATOR));
    DRIVER_CONTROLLER.leftTrigger(0.2).whileTrue(
      new ConditionalCommand(
        new LobNote(SHOOTER, WRIST, ELEVATOR),
        new PointAtAprilTag(
                DRIVETRAIN,
                () -> -TranslationXSlewRate.calculate(DRIVER_CONTROLLER.getLeftY()),
                () -> -TranslationYSlewRate.calculate(DRIVER_CONTROLLER.getLeftX()),
                () -> DRIVER_CONTROLLER.getRightX(),
                () -> true), 
                () -> DRIVER_CONTROLLER.getLeftTriggerAxis() > 0.95)
    );
    
    // WIP
    // DRIVER_CONTROLLER
    //     .leftTrigger()
    //     .onTrue(
    //         new ParallelDeadlineGroup(
    //                 new ParallelCommandGroup(
    //                     new WaitUntilCommand(
    //                         () -> {
    //                           Pose2d currentPose = DRIVETRAIN.getPose();
    //                           Rotation2d currentRotation = currentPose.getRotation();
    //                           DriverStation.reportWarning(
    //                               "current: "
    //                                   + currentRotation.getDegrees()
    //                                   + ", target: "
    //                                   + Util.getRotationToAllianceLob(currentPose).getDegrees(),
    //                               false);
    //                           return Util.isWithinTolerance(
    //                               currentRotation.getDegrees(),
    //                               Util.getRotationToAllianceLob(currentPose).getDegrees(),
    //                               30);
    //                         }),
    //                     new PrintCommand("STARTING LOB SEQUENCE")),
    //                 new PointAtAprilTag(
    //                     DRIVETRAIN,
    //                     () -> -TranslationXSlewRate.calculate(DRIVER_CONTROLLER.getLeftY()),
    //                     () -> -TranslationYSlewRate.calculate(DRIVER_CONTROLLER.getLeftX()),
    //                     () -> DRIVER_CONTROLLER.getRightX(),
    //                     true))
    //             .andThen(new LobNote(SHOOTER, WRIST, ELEVATOR)))
    //     .onFalse(new InstantCommand());
  }

  private void configureCoDriverController() {
    CO_DRIVER_CONTROLLER
        .back()
        .onTrue(
            Util.pathfindToPose(Util.getAllianceAmp())
                .andThen(
                    new PrepRevAmp(ELEVATOR, WRIST)
                        .andThen(new InstantCommand(() -> isAmpPrepped = true))));
    CO_DRIVER_CONTROLLER.start().onTrue(new StopAll(WRIST, SHOOTER, INTAKE, ELEVATOR));
    CO_DRIVER_CONTROLLER.rightBumper().onTrue(new PoopNote(SHOOTER, 2500));

    CO_DRIVER_CONTROLLER
        .y()
        .onTrue(
            new ConditionalCommand(
                new ConditionalCommand(
                    new Climb(ELEVATOR, WRIST, SHOOTER),
                    new InstantCommand(
                            () -> {
                              ELEVATOR.setLengthInches(Constants.Climb.CLIMB_START_HEIGHT);
                              WRIST.goHome();
                            },
                            ELEVATOR,
                            WRIST)
                        .andThen(() -> isClimbPrimed = true),
                    () -> isClimbPrimed),
                new InstantCommand(),
                () -> DriverStation.getMatchTime() < 45.0));

    CO_DRIVER_CONTROLLER.x().onTrue(new ReverseIntake(SHOOTER, INTAKE, WRIST, ELEVATOR));
    CO_DRIVER_CONTROLLER.leftTrigger().onTrue(new ShootTall(ELEVATOR, WRIST, SHOOTER));
    CO_DRIVER_CONTROLLER
        .b()
        .onTrue(
            new InstantCommand(
                    () -> {
                      SHOOTER.setFeederVoltage(Constants.Shooter.FEEDER_FEEDFWD_VOLTS);
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

    CO_DRIVER_CONTROLLER.rightTrigger().onTrue(new ShootSubwooferFlat(ELEVATOR, WRIST, SHOOTER));
    CO_DRIVER_CONTROLLER
        .a()
        .onTrue(
            new ParallelDeadlineGroup(
                new IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR),
                new DriveToNoteAuto(DRIVETRAIN, INTAKE_PHOTON, SHOOTER, INTAKE, WRIST, ELEVATOR)));
  }

  private void configureDrivetrain() {

    DRIVETRAIN.setDefaultCommand( // Drivetrain will execute this command periodically
        new SwerveCommand(
            DRIVETRAIN,
            drive,
            () -> -TranslationXSlewRate.calculate(DRIVER_CONTROLLER.getLeftY()),
            () -> -TranslationYSlewRate.calculate(DRIVER_CONTROLLER.getLeftX()),
            () -> DRIVER_CONTROLLER.getRightX(),
            () -> DRIVER_CONTROLLER.getHID().getPOV()));

    // DRIVETRAIN.setDefaultCommand( // Drivetrain will execute this command periodically
    //     DRIVETRAIN
    //         .applyRequest(
    //             () ->
    //                 drive
    //                     .withVelocityX(
    //                         Util.squareValue(-DRIVER_CONTROLLER.getLeftY())
    //                             * DriveConstants.kSpeedAt12VoltsMps) // Drive forward with
    //                     // negative Y (forward)
    //                     .withVelocityY(
    //                         Util.squareValue(-DRIVER_CONTROLLER.getLeftX())
    //                             * DriveConstants
    //                                 .kSpeedAt12VoltsMps) // Drive left with negative X (left)
    //                     .withRotationalRate(
    //                         Util.squareValue(-DRIVER_CONTROLLER.getRightX())
    //                             * Math.PI
    //                             * 3.5) // Drive counterclockwise with negative X (left)
    //             )
    //         .ignoringDisable(true));

    // DRIVETRAIN.setDefaultCommand(
    //     new TeleopSwerve(
    //         DRIVETRAIN,
    //         () -> -DRIVER_CONTROLLER.getLeftY(),
    //         () -> -DRIVER_CONTROLLER.getLeftX(),
    //         DRIVER_CONTROLLER::getRightX,
    //         () -> 1.0,
    //         () -> DRIVER_CONTROLLER.getHID().getPOV(),
    //         () -> false));

    if (Utils.isSimulation()) {
      DRIVETRAIN.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    DRIVETRAIN.registerTelemetry(logger::telemeterize);
  }

  double highest = 0.0;

  public double getAverageSpeeds() {
    double[] speeds =
        new double[] {
          DRIVETRAIN.getModule(0).getDriveMotor().getRotorVelocity().getValueAsDouble(),
          DRIVETRAIN.getModule(1).getDriveMotor().getRotorVelocity().getValueAsDouble(),
          DRIVETRAIN.getModule(2).getDriveMotor().getRotorVelocity().getValueAsDouble(),
          DRIVETRAIN.getModule(3).getDriveMotor().getRotorVelocity().getValueAsDouble()
        };
    double total = 0.0;
    double avg = 0.0;

    for (double d : speeds) {
      total = total + Math.abs(d);
    }
    avg = total / 4.0;
    if (avg > highest) {
      highest = avg;
    }
    return highest;
  }

  public void configureShuffleboard() {
    WRIST.setupShuffleboard();
    SHOOTER.setupShuffleboard();
    INTAKE.setupShuffleboard();
    ELEVATOR.setupShuffleboard();
    COMMANDS_TAB.addDouble("AVG SPEED 4 MODULES", () -> getAverageSpeeds());
    COMMANDS_TAB.add("Stop Logger", new InstantCommand(() -> SignalLogger.stop()));
    COMMANDS_TAB.add(
        "Unjam Shooter",
        new InstantCommand(
                () -> {
                  SHOOTER.setFeederVoltage(12);
                  SHOOTER.setShooterVoltage(12);
                })
            .andThen(new WaitCommand(1.0))
            .andThen(
                new InstantCommand(
                    () -> {
                      SHOOTER.setFeederVoltage(-12);
                      SHOOTER.setShooterVoltage(-12);
                    }))
            .andThen(
                new InstantCommand(
                    () -> {
                      SHOOTER.stopFeeder();
                      SHOOTER.stopShooter();
                    })));
    PHOTON_TAB.addDouble("DISTANCE_TO_SPEAKER", () -> Util.getDistanceToSpeaker());

    COMMANDS_TAB.add(
        "Coast Swerve",
        new InstantCommand(
                () -> {
                  for (int i = 0; i < 4; i++) {
                    DRIVETRAIN.getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
                    DRIVETRAIN.getModule(i).getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
                  }
                })
            .ignoringDisable(true));
    COMMANDS_TAB.add("ShootAmp", new ShootAmp(SHOOTER, ELEVATOR, WRIST));
    COMMANDS_TAB.add(
        "Brake Swerve",
        new InstantCommand(
                () -> {
                  for (int i = 0; i < 3; i++) {
                    DRIVETRAIN.getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
                    DRIVETRAIN.getModule(i).getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
                  }
                })
            .ignoringDisable(true));
    COMMANDS_TAB.add("Lob Note", new LobNote(SHOOTER, WRIST, ELEVATOR));
    // COMMANDS_TAB.add("Fast Point", new FastPoint(DRIVETRAIN, SPEAKER_LIMELIGHT));
    COMMANDS_TAB.add("NewShootAMpAuto", new NewShootAmpAuto(SHOOTER, ELEVATOR, WRIST));
    COMMANDS_TAB.add(
        "Force Zero All",
        new InstantCommand(
                () -> {
                  ELEVATOR.setZero();
                  WRIST.setZero();
                })
            .ignoringDisable(true));
    COMMANDS_TAB.add(
        "SetPoseRedSubStart",
        new InstantCommand(
                () ->
                    DRIVETRAIN.seedFieldRelative(
                        new Pose2d(15.2, 5.5, Rotation2d.fromDegrees(-180))))
            .ignoringDisable(true));

    AUTO_CHOOSER.addOption(
        "SYSID-QUAS-F", DRIVETRAIN.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    AUTO_CHOOSER.addOption(
        "SYSID-QUAS-R", DRIVETRAIN.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    AUTO_CHOOSER.addOption("SYSID-DYN-R", DRIVETRAIN.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    AUTO_CHOOSER.addOption("SYSID-DYN-F", DRIVETRAIN.sysIdDynamic(SysIdRoutine.Direction.kForward));

    // PHOTON_TAB.add(
    //     "Square Up AprilTag",
    //     new SquareUpToAprilTag(DRIVETRAIN, SPEAKER_LIMELIGHT, Constants.SPEAKER_APRILTAG_HEIGHT,
    // 3));
    PHOTON_TAB.add(
        "Drive To Note",
        new DriveToNote(DRIVETRAIN, () -> -DRIVER_CONTROLLER.getLeftY(), INTAKE_PHOTON));
    PHOTON_TAB.add(
        "Drive To Note Auto",
        new DriveToNoteAuto(DRIVETRAIN, INTAKE_PHOTON, SHOOTER, INTAKE, WRIST, ELEVATOR));
    PHOTON_TAB.add(
        "Collect Note Auto",
        new CollectNoteAuto(DRIVETRAIN, SHOOTER, INTAKE, WRIST, ELEVATOR, INTAKE_PHOTON));

    SHOOTER_TAB.add("Spit Note", new SpitNote(SHOOTER));
    // MATCH_TAB.add(
    //     "Test InstantShoot",
    //     new ParallelCommandGroup(
    //         new InstantCommand(() -> WRIST.setDegrees(20.0)),
    //         new AutoIdleShooter(SHOOTER),
    //         new WaitCommand(1.0).andThen(new InstantShoot(SHOOTER))));

    // addAuto("ampa-full");
    // addAuto("sourcea-fullshoot");
    // addAuto("amp_close");
    // addAuto("amp_subwoofer_reversal");
    // addAuto("driven_source_score");
    // addAuto("heart_source_shoot");
    // addAuto("heart_source_og");
    // addAuto("Taxi-Amp");
    // addAuto("Taxi-Source");
    // addAuto("temp_center");
    // addAuto("GKC-SOURCE-A");
    // addAuto("GKC-SOURCE-B");
    // addAuto("GKC-AMP-A");
    // addAuto("GKC-AMP-B");
    // addAuto("GKC-Source-J");
    // addAuto("GKC Source 5-4-3");
    // addAuto("GKC Source 4-3-2");
    // addAuto("GKC-Amp-J");
    // addAuto("GKC-Amp-2-1Blue");
    // addAuto("GKC-Amp-2-1Red");
    // addAuto("GKC-Amp-1-2Blue");
    // addAuto("GKC-Amp-1-2Red");
    // addAuto("AGKC-Amp-1-2Red");
    // addAuto("GKC-Amp-Skip-1-2");
    addAuto("Middle Race Cleanup");
    addAuto("lame");
    // AUTO_CHOOSER.addOption(
    //     "Middle Race Cleanup",
    //     new MiddleRaceCleanup(DRIVETRAIN, INTAKE, ELEVATOR, WRIST, SHOOTER, INTAKE_PHOTON));
    // addAuto("GKC-Amp-J2");
    AUTO_CHOOSER.addOption("Do Nothing", new InstantCommand());
    MATCH_TAB.add("Auto", AUTO_CHOOSER);
    MATCH_TAB.add(
        "Full confidence",
        new InstantCommand(
            () ->
                DRIVETRAIN.addVisionMeasurement(
                    LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-leftlo").pose,
                    LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-leftlo")
                        .timestampSeconds,
                    VecBuilder.fill(0.1, 0.1, 0.1))));
    // MATCH_TAB.addBoolean("Vision Updated", () -> VisionUpdate);

    // MATCH_TAB.addBoolean("is Alliance red", () -> CommandSwerveDrivetrain.getAlliance() ==
    // Alliance.Red ? true : false);
  }

  public void configureDefaultCommands() {
    WRIST.setDefaultCommand(new AimLockWrist(WRIST, SHOOTER, ELEVATOR));
    SHOOTER.setDefaultCommand(new IdleShooter(SHOOTER));
    CANDLES.setDefaultCommand(new DefaultCANdle(CANDLES, SHOOTER));
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand(
        "SubwooferSecondHalf",
        new SequentialCommandGroup(
            new WaitCommand(0.04),
            new InstantCommand(
                () -> {
                  ELEVATOR.setLengthInches(0.5);
                  WRIST.setDegrees(12);
                  SHOOTER.stopShooter();
                  SHOOTER.stopFeeder();
                })));
    NamedCommands.registerCommand(
        "ShootNote",
        new ParallelDeadlineGroup(
                new ShootNote(SHOOTER, ELEVATOR, Constants.Shooter.SHOOTER_RPM),
                new AimLockWrist(WRIST, SHOOTER, ELEVATOR),
                new InstantCommand(() -> aimAtTargetAuto = true))
            .andThen(new InstantCommand(() -> aimAtTargetAuto = false)));
    NamedCommands.registerCommand(
        "ShootNoteRegular",
        new ShootNoteAimbotFixed(SHOOTER, ELEVATOR, Constants.Shooter.SHOOTER_RPM, WRIST)
            .withTimeout(3.0)
            .andThen(new PoopNote(SHOOTER, 1000).withTimeout(0.7)));
    NamedCommands.registerCommand(
        "ShootNoteAimbot",
        new ShootNoteSequence(SHOOTER, WRIST, Constants.Shooter.SHOOTER_RPM, DRIVETRAIN));
    NamedCommands.registerCommand(
        "SpinUpShooter", new InstantCommand(() -> SHOOTER.setRPMShoot(5200)));

    NamedCommands.registerCommand(
        "SpeedUpShooter",
        new InstantCommand(() -> SHOOTER.setRPMShoot(Constants.Shooter.SHOOTER_IDLE_RPM_CLOSE)));
    NamedCommands.registerCommand(
        "IntakeNote",
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new AimLockWrist(WRIST, SHOOTER, ELEVATOR),
                new IntakeNoteSequence(SHOOTER, INTAKE, ELEVATOR, false, -7)),
            new InstantCommand(
                () -> SHOOTER.setRPMShoot(Constants.Shooter.SHOOTER_IDLE_RPM_CLOSE))));
    NamedCommands.registerCommand(
        "IntakeNoteSlow",
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new AimLockWrist(WRIST, SHOOTER, ELEVATOR),
                new IntakeNoteSequence(SHOOTER, INTAKE, ELEVATOR, false, -4)),
            new InstantCommand(
                () -> SHOOTER.setRPMShoot(Constants.Shooter.SHOOTER_IDLE_RPM_CLOSE))));
    NamedCommands.registerCommand(
        "PoopPrep",
        new InstantCommand(
            () -> {
              SHOOTER.setRPMShootNoSpin(750);
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
              SHOOTER.setRPMShootNoSpin(4000);
              WRIST.setDegrees(25);
            },
            SHOOTER,
            WRIST,
            INTAKE));
    NamedCommands.registerCommand(
        "PoopStart", new InstantCommand(() -> SHOOTER.setFeederVoltage(8.0)));
    NamedCommands.registerCommand(
        "StartIntakeAndShoot",
        new InstantCommand(
            () -> {
              SHOOTER.setFeederVoltage(Constants.Shooter.FEEDER_AUTO_VOLTS);
              INTAKE.setRPM(Constants.INTAKE_RPM);
            },
            INTAKE));
    NamedCommands.registerCommand(
        "StopIntakeAndShoot",
        new InstantCommand(
            () -> {
              SHOOTER.setFeederVoltage(0.0);
              INTAKE.setRPM(0.0);
            }));
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
    NamedCommands.registerCommand("GoHome", new GoHome(ELEVATOR, WRIST, SHOOTER, INTAKE));
    NamedCommands.registerCommand("NewShootAmp", new NewShootAmpAuto(SHOOTER, ELEVATOR, WRIST));
    NamedCommands.registerCommand(
        "DriveToNoteAuto",
        new ParallelDeadlineGroup(
            new IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR),
            new DriveToNoteAuto(DRIVETRAIN, INTAKE_PHOTON, SHOOTER, INTAKE, WRIST, ELEVATOR)));
    NamedCommands.registerCommand(
        "OverrideRotationSpeakerEnable", new InstantCommand(() -> aimAtTargetAuto = true));
    NamedCommands.registerCommand(
        "OverrideRotationSpeakerDisable", new InstantCommand(() -> aimAtTargetAuto = false));
    NamedCommands.registerCommand("DefaultWrist", new AutoAimLockWrist(WRIST));
    NamedCommands.registerCommand("DefaultShooter", new AutoIdleShooter(SHOOTER));
    NamedCommands.registerCommand("InstantShoot", new InstantShoot(SHOOTER));
    NamedCommands.registerCommand(
        "IntakeNoteAuto", new IntakeNoteSequenceAuto(SHOOTER, INTAKE, ELEVATOR));
    NamedCommands.registerCommand(
        "TempLog",
        new InstantCommand(
            () ->
                DriverStation.reportWarning(
                    "InstantShoot: distance: "
                        + Util.getDistanceToSpeaker()
                        + ", angle: "
                        + Util.getInterpolatedWristAngle(),
                    false)));
    NamedCommands.registerCommand(
        "AutoCollectNote",
        new AutoCollectNote(DRIVETRAIN, INTAKE_PHOTON, 2.0, SHOOTER, INTAKE, ELEVATOR));
    NamedCommands.registerCommand("AutoAimAndShoot", new AutoAimAndShoot(DRIVETRAIN, SHOOTER));
    NamedCommands.registerCommand("EnableWristLockDown", new InstantCommand(() -> WRIST.enableWristLockdown()));
    NamedCommands.registerCommand("DisableWristLockDown", new InstantCommand(() -> WRIST.disableWristLockdown()));
  }

  public void addAuto(String autoName) {
    final PathPlannerAuto auto = new PathPlannerAuto(autoName);
    if (autoName == "GKC-Amp-2-1Blue"
        || autoName == "GKC-Amp-1-2Blue"
        || autoName == "GKC-Amp-1-2Red"
        || autoName == "AGKC-Amp-1-2Red"
        || autoName == "GKC-Amp-Skip-1-2"
        || autoName == "GKC-Amp-2-1Red") {
      AUTO_CHOOSER.addOption(autoName, new ShootSubwoofer(ELEVATOR, WRIST, SHOOTER).andThen(auto));
      return;
    } else if (autoName == "GKC Source 5-4-3"
        || autoName == "GKC Source 4-3-2"
        || autoName == "Middle Race Cleanup") {
      AUTO_CHOOSER.addOption(
          autoName, new ShootSubwooferFirstHalf(ELEVATOR, WRIST, SHOOTER).andThen(auto));
      return;
    }
    AUTO_CHOOSER.addOption(autoName, auto);
  }

  public Command getAutonomousCommand() {

    return AUTO_CHOOSER.getSelected();
  }

  public Pose2d getPose() {
    return DRIVETRAIN.getPose();
  }

  public boolean shouldRejectLL3G(final LimelightHelpers.PoseEstimate botPose) {
    if (botPose.tagCount < 2) {
      return true;
    }

    if (SHOOTER.isFeeding()) {
      return true;
    }
    // if (botPose.tagCount == 1 && botPose.rawFiducials[0].ambiguity > 0.9) {
    //   return true;
    // }
    if (botPose.tagCount == 2 && botPose.avgTagDist > 4.0) { // 4.0
      return true;
    }
    // Reject a pose outside of the field.
    if (!Constants.Vision.fieldBoundary.isPoseWithinArea(botPose.pose)) {
      return true;
    }
    return false;
  }

  public void updatePoseVision() {
    updatePoseVision(!DriverStation.isDisabled() ? 0.2 : 99999, true);
  }

  public void updatePoseVision(double rotationConfidence, boolean canTrustLL3) {
    ChassisSpeeds currentSpeed = DRIVETRAIN.getCurrentRobotChassisSpeeds();
    // Reject pose updates when moving fast.
    if (Math.abs(currentSpeed.vxMetersPerSecond) > 1.5
        || Math.abs(currentSpeed.vyMetersPerSecond) > 1.5
        || Math.abs(Math.toDegrees(currentSpeed.omegaRadiansPerSecond)) > 180.0) {
      // DriverStation.reportWarning("Ignoring Pose update due to speed", false);
      return;
    }
    boolean isDisabled = DriverStation.isDisabled();
    boolean hasUpdatedPose = false;

    LimelightHelpers.PoseEstimate bestPose = null;
    double bestConfidence = 0.0;

    for (String limelight3G : Constants.Vision.LL3GS) {
      LimelightHelpers.PoseEstimate botPoseLL3G =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight3G);
      if (!shouldRejectLL3G(botPoseLL3G)) {
        double currentConfidence = calculateConfidence(botPoseLL3G);
        if (currentConfidence > bestConfidence) {
          bestConfidence = currentConfidence;
          bestPose = botPoseLL3G;
          // System.out.println("UPDATING POSE LL3G");
          // System.out.println(botPoseLL3G);
        }
      } else {
        // System.out.println("REJECTED POSE LL3G " + limelight3G);
      }
    }
    if (bestConfidence > 0.0) {
      updatePose(bestPose, isDisabled, rotationConfidence);
      hasUpdatedPose = true;
    }
    if (canTrustLL3) {
      if (!hasUpdatedPose) {
        for (String limelight3 : Constants.Vision.LL3S) {
          LimelightHelpers.PoseEstimate botPoseLL3 =
              LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight3);
          if (!shouldRejectLL3G(botPoseLL3)) {
            hasUpdatedPose = true;
            updatePose(botPoseLL3, isDisabled, rotationConfidence);
            // DriverStation.reportWarning("UPDATING POSE LL3", false);
          } else {
            // DriverStation.reportWarning("REJECTED POSE LL3 " + limelight3, false);
          }
        }
      }
    }

    if (!hasUpdatedPose) {
      // System.out.println("FAILED TO UPDATE POSE!");
    }
  }

  public void updatePose(
      LimelightHelpers.PoseEstimate botPose, boolean isDisabled, double rotationConfidence) {
    double currentConfidence = calculateConfidence(botPose);
    if (DriverStation.isAutonomous()) {
      currentConfidence = currentConfidence * 6;
    }
    // if (VisionUpdate) {
    //   VisionUpdate = false;
    // } else {
    //   VisionUpdate = true;
    // }
    DRIVETRAIN.addVisionMeasurement(
        botPose.pose,
        botPose.timestampSeconds,
        VecBuilder.fill(
            currentConfidence, currentConfidence, rotationConfidence * botPose.tagCount));
  }

  // Reference FRC 6391
  // https://github.com/6391-Ursuline-Bearbotics/2024-6391-Crescendo/blob/4759dfd37c960cf3493b0eafd901519c5c36b239/src/main/java/frc/robot/Vision/Limelight.java#L44-L88
  // public void updatePoseVision(final String... limelights) {
  //   ChassisSpeeds currentSpeed = DRIVETRAIN.getCurrentRobotChassisSpeeds();
  //   // Reject pose updates when moving fast.
  //   if (Math.abs(currentSpeed.vxMetersPerSecond) > 2.5
  //       || Math.abs(currentSpeed.vyMetersPerSecond) > 2.5
  //       || Math.abs(Math.toDegrees(currentSpeed.omegaRadiansPerSecond)) > 270.0) {
  //     DriverStation.reportWarning("Ignoring Pose update due to speed", false);
  //     return;
  //   }

  //   double rotationConfidence = DriverStation.isDisabled() ? 0.2 : 999;

  //   for (String limelight : limelights) {
  //     Limelight.PoseEstimate botPose = Limelight.getBotPoseEstimate_wpiBlue(limelight);

  //     // Reject an empty pose or a pose with no AprilTags.
  //     if (botPose.tagCount < 1) {
  //       continue;
  //     }
  //     if (botPose.tagCount == 1 && botPose.rawFiducials[0].ambiguity > 0.9) {
  //       continue;
  //     }
  //     if (botPose.tagCount == 2 && botPose.avgTagDist > 4.5) {
  //       continue;
  //     }
  //     // if (botPose.tagCount == 3 && botPose.avgTagDist > 5.5) {
  //     //   continue;
  //     // }

  //     // Reject a pose outside of the field.
  //     if (!Constants.Vision.fieldBoundary.isPoseWithinArea(botPose.pose)) {
  //       continue;
  //     }

  //     // final double botPoseToPoseDistance =
  //     // botPose.pose.getTranslation().getDistance(DRIVETRAIN.getPose().getTranslation());
  //     // if (botPoseToPoseDistance > 2 && !DriverStation.isDisabled()) {
  //     //   System.out.println("Rejecting, Bot pose and limelight pose to far");
  //     //   continue;
  //     // }

  //     final double botPoseToPoseDistance =
  //         botPose.pose.getTranslation().getDistance(DRIVETRAIN.getPose().getTranslation());
  //     if (botPose.tagCount < 2 && botPoseToPoseDistance > 1.5) {
  //       continue;
  //     }
  //     double currentConfidence = calculateConfidence(botPose);
  //     DRIVETRAIN.addVisionMeasurement(
  //         botPose.pose,
  //         botPose.timestampSeconds,
  //         VecBuilder.fill(
  //             currentConfidence, currentConfidence, rotationConfidence * botPose.tagCount));
  //     // if (currentConfidence > bestConfidence) {
  //     //   bestConfidence = currentConfidence;
  //     //   bestPose = botPose;
  //     //   // System.out.println("Updating pose with: " + limelight);
  //     // }
  //   }

  //   // if (bestPose == null) {
  //   //   return; // No valid pose found
  //   // }
  //   // if (DriverStation.isDisabled() && (
  //   //   (bestPose.tagCount == 1 && bestPose.rawFiducials[0].ambiguity < 0.9)
  //   //   || (bestPose.tagCount == 2 && bestPose.avgTagDist < 4)
  //   //   || (bestPose.tagCount > 2))
  //   // ) {
  //   //   DRIVETRAIN.addVisionMeasurement(
  //   //       bestPose.pose, bestPose.timestampSeconds, VecBuilder.fill(0.2, 0.2, 1));
  //   // } else {

  //   //   // System.out.println("Best Confidence Value: " + (bestConfidence + 0.7));
  //   //   DRIVETRAIN.addVisionMeasurement(
  //   //       bestPose.pose,
  //   //       bestPose.timestampSeconds,
  //   //       VecBuilder.fill(bestConfidence + 0.7, bestConfidence + 0.7, 99999999));
  //   // }
  // }

  private double calculateConfidence(LimelightHelpers.PoseEstimate botPose) {
    // final double botPoseToPoseDistance =
    //       botPose.pose.getTranslation().getDistance(DRIVETRAIN.getPose().getTranslation());
    final double speakerReference = Util.speakerTagCount(botPose.rawFiducials);
    final double distanceWeight =
        Math.pow(botPose.avgTagDist / Constants.Vision.MAX_DISTANCE_SCALING, 2);
    // System.out.println(distanceWeight);
    if (botPose.tagCount > 2) {
      return (0.7 / speakerReference) + distanceWeight;
    }
    if (botPose.tagCount == 2) {
      return (0.9 / speakerReference) + distanceWeight;
    }
    if (botPose.tagCount == 1) {
      return (2.2 / speakerReference) + distanceWeight;
    }
    return 99999;
  }

  // private double calculateConfidence(Limelight.PoseEstimate botPose) {
  //   double confidence = 0.0;
  //   double tagDistanceFeet = Units.metersToFeet(botPose.avgTagDist);

  //   if (botPose.tagCount > 1) {
  //     confidence = 1.0 / tagDistanceFeet;
  //   } else if (botPose.tagCount == 1) {
  //     tagDistanceFeet *= 2; // Assume less precision with single tag.
  //     confidence = 0.5 / tagDistanceFeet;
  //   }

  //   if (!DriverStation.isDisabled() && tagDistanceFeet < 15) {
  //     final double botPoseToPoseDistance =
  //         botPose.pose.getTranslation().getDistance(DRIVETRAIN.getPose().getTranslation());
  //     confidence = confidence + Math.pow(botPoseToPoseDistance / 3, 2);
  //   }

  //   // Ensure confidence stays within a reasonable range.
  //   // confidence = Math.max(0.1, Math.min(0.7, 99));

  //   return confidence;
  // }

  public static RobotContainer get() {
    return instance;
  }
}
