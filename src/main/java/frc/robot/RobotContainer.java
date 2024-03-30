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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.commands.control.Climb;
import frc.robot.commands.control.GoHome;
import frc.robot.commands.control.IdleShooter;
import frc.robot.commands.control.NewShootAmpAuto;
import frc.robot.commands.control.ReverseIntake;
import frc.robot.commands.control.ShootAmp;
import frc.robot.commands.control.ShootSubwoofer;
import frc.robot.commands.control.ShootSubwooferFlat;
import frc.robot.commands.control.ShootTall;
import frc.robot.commands.control.StopAll;
import frc.robot.commands.control.amp.FireRevAmp;
import frc.robot.commands.control.amp.PrepRevAmp;
import frc.robot.commands.control.auto.AutoAimLockWrist;
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
import frc.robot.subsystems.Candles;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.util.Limelight;
import frc.robot.util.Util;

public class RobotContainer {
  private final SendableChooser<Command> AUTO_CHOOSER = new SendableChooser<>();
  public final ShuffleboardTab COMMANDS_TAB = Shuffleboard.getTab("COMMANDS");
  public final ShuffleboardTab MATCH_TAB = Shuffleboard.getTab("MATCH");
  public final ShuffleboardTab PHOTON_TAB = Shuffleboard.getTab("PHOTON");
  public final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("SHOOTER");
  public final ShuffleboardTab PROTO_TAB = Shuffleboard.getTab("PROTO");
  public final String SPEAKER_LIMELIGHT = "limelight-speaker";
  public final String RIGHT_LIMELIGHT = "limelight-right";
  public final String AMP_LIMELIGHT = "limelight-amp";
  public final Vision INTAKE_PHOTON = new Vision("Arducam_OV9782_USB_Camera", 0.651830, 60);
  public final CommandSwerveDrivetrain DRIVETRAIN = DriveConstants.DriveTrain; // My drivetrain
  public final Candles CANDLES = new Candles(Constants.LEFT_CANDLE, Constants.RIGHT_CANDLE);
  public final Intake INTAKE = new Intake(Constants.INTAKE_TOP_ID, Constants.INTAKE_BOTTOM_ID);
  public final Shooter SHOOTER =
      new Shooter(
          Constants.SHOOTER_LEADER_ID, Constants.SHOOTER_FOLLOWER_ID, Constants.SHOOTER_FEEDER_ID);
  public final Wrist WRIST = new Wrist(Constants.WRIST_ID);
  public final Elevator ELEVATOR =
      new Elevator(Constants.ELEVATOR_LEADER_ID, Constants.ELEVATOR_FOLLOWER_ID);
  private final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(0);
  private final CommandXboxController CO_DRIVER_CONTROLLER = new CommandXboxController(1);
  public static boolean isForwardAmpPrimed = false;
  public static boolean isReverseAmpPrimed = false;
  public static boolean isClimbPrimed = false;
  public static boolean aimAtTargetAuto = false;
  private double MaxSpeed = DriveConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = 1.5 * Math.PI;

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

    DRIVER_CONTROLLER
        .y()
        .onTrue(
            new ConditionalCommand(
                new GoHome(ELEVATOR, WRIST, SHOOTER, INTAKE)
                    .andThen(() -> isReverseAmpPrimed = false),
                new PrepRevAmp(ELEVATOR, WRIST)
                    .andThen(new WaitCommand(0.8))
                    .andThen(new FireRevAmp(SHOOTER))
                    .andThen(new WaitCommand(0.1))
                    .andThen(new InstantCommand(() -> ELEVATOR.setLengthInches(4.2)))
                    .andThen(new InstantCommand(() -> isReverseAmpPrimed = true)),
                () -> isReverseAmpPrimed));
    DRIVER_CONTROLLER
        .back()
        .onTrue(
            DRIVETRAIN.runOnce(
                () -> {
                  DRIVETRAIN.seedFieldRelative();
                  DRIVETRAIN.getPigeon2().reset();
                }));
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
                    new AsyncRumble(
                        DRIVER_CONTROLLER.getHID(), RumbleType.kBothRumble, 1.0, 700L)));

    DRIVER_CONTROLLER
        .rightTrigger()
        .whileTrue(
            new PointAtAprilTag(
                DRIVETRAIN,
                SPEAKER_LIMELIGHT,
                () -> (DRIVER_CONTROLLER.getLeftY()),
                () -> (DRIVER_CONTROLLER.getLeftX()),
                () -> (DRIVER_CONTROLLER.getRightX())));

    DRIVER_CONTROLLER
        .rightBumper()
        .onTrue(new ShootNote(SHOOTER, ELEVATOR, Constants.Shooter.SHOOTER_RPM));
    DRIVER_CONTROLLER.leftTrigger().onTrue(new LobNote(SHOOTER, WRIST, ELEVATOR));
  }

  private void configureCoDriverController() {
    CO_DRIVER_CONTROLLER
        .leftBumper()
        .onTrue(Util.pathfindToPose(Util.findNearestPoseToTrapClimbs(getPose())));
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
            () -> DRIVER_CONTROLLER.getLeftY(),
            () -> DRIVER_CONTROLLER.getLeftX(),
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

    COMMANDS_TAB.addBoolean("Can See Target", () -> Util.canSeeTarget(SPEAKER_LIMELIGHT));
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

    addAuto("ampa-full");
    addAuto("sourcea-fullshoot");
    addAuto("amp_close");
    addAuto("amp_subwoofer_reversal");
    addAuto("driven_source_score");
    addAuto("heart_source_shoot");
    addAuto("heart_source_og");
    addAuto("Taxi-Amp");
    addAuto("Taxi-Source");
    addAuto("temp_center");
    addAuto("GKC-SOURCE-A");
    addAuto("GKC-SOURCE-B");
    addAuto("GKC-AMP-A");
    addAuto("GKC-AMP-B");
    addAuto("GKC-Source-J");
    addAuto("GKC-Amp-J");
    AUTO_CHOOSER.addOption("Do Nothing", new InstantCommand());
    MATCH_TAB.add("Auto", AUTO_CHOOSER);
  }

  public void configureDefaultCommands() {
    WRIST.setDefaultCommand(new AimLockWrist(WRIST, SHOOTER, ELEVATOR, SPEAKER_LIMELIGHT));
    SHOOTER.setDefaultCommand(new IdleShooter(SHOOTER, SPEAKER_LIMELIGHT));
    CANDLES.setDefaultCommand(new DefaultCANdle(CANDLES, SHOOTER, SPEAKER_LIMELIGHT));
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand(
        "ShootNote",
        new ParallelDeadlineGroup(
                new ShootNote(SHOOTER, ELEVATOR, Constants.Shooter.SHOOTER_RPM),
                new AimLockWrist(WRIST, SHOOTER, ELEVATOR, SPEAKER_LIMELIGHT),
                new InstantCommand(() -> aimAtTargetAuto = true))
            .andThen(new InstantCommand(() -> aimAtTargetAuto = false)));
    NamedCommands.registerCommand(
        "ShootNoteRegular",
        new ShootNoteAimbotFixed(
                SHOOTER, ELEVATOR, Constants.Shooter.SHOOTER_RPM, SPEAKER_LIMELIGHT, WRIST)
            .withTimeout(3.0)
            .andThen(new PoopNote(SHOOTER, 1000).withTimeout(0.7)));
    NamedCommands.registerCommand(
        "ShootNoteAimbot",
        new ShootNoteSequence(
            SHOOTER, WRIST, Constants.Shooter.SHOOTER_RPM, DRIVETRAIN, SPEAKER_LIMELIGHT));
    NamedCommands.registerCommand(
        "SpinUpShooter", new InstantCommand(() -> SHOOTER.setRPMShoot(5200)));

    NamedCommands.registerCommand(
        "SpeedUpShooter",
        new InstantCommand(() -> SHOOTER.setRPMShoot(Constants.Shooter.SHOOTER_IDLE_RPM_CLOSE)));
    NamedCommands.registerCommand(
        "IntakeNote",
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new AimLockWrist(WRIST, SHOOTER, ELEVATOR, SPEAKER_LIMELIGHT),
                new IntakeNoteSequence(SHOOTER, INTAKE, ELEVATOR, false, -7)),
            new InstantCommand(
                () -> SHOOTER.setRPMShoot(Constants.Shooter.SHOOTER_IDLE_RPM_CLOSE))));
    NamedCommands.registerCommand(
        "IntakeNoteSlow",
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new AimLockWrist(WRIST, SHOOTER, ELEVATOR, SPEAKER_LIMELIGHT),
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
        "IntakeNoteAuto",
        new IntakeNoteSequenceAuto(
            SHOOTER, INTAKE, WRIST,
            ELEVATOR)); // new InstantCommand(() -> aimAtTargetAuto = true)).andThen()
    NamedCommands.registerCommand("c", new ShootSubwoofer(ELEVATOR, WRIST, SHOOTER).asProxy());
  }

  public void addAuto(String autoName) {
    AUTO_CHOOSER.addOption(autoName, new PathPlannerAuto(autoName));
  }

  public Command getAutonomousCommand() {

    return AUTO_CHOOSER.getSelected();
  }

  public Pose2d getPose() {
    return DRIVETRAIN.getPose();
  }

  // Reference FRC 6391
  // https://github.com/6391-Ursuline-Bearbotics/2024-6391-Crescendo/blob/4759dfd37c960cf3493b0eafd901519c5c36b239/src/main/java/frc/robot/Vision/Limelight.java#L44-L88
  public void updatePoseVision(final String limelight) {
    ChassisSpeeds currentSpeed = DRIVETRAIN.getCurrentRobotChassisSpeeds();
    // Reject pose updates when moving fast.
    if (Math.abs(currentSpeed.vxMetersPerSecond) > 2.0
        || Math.abs(currentSpeed.vyMetersPerSecond) > 2.0
        || Math.abs(currentSpeed.omegaRadiansPerSecond) > Math.PI) {
          DriverStation.reportWarning("Ignoring Pose update due to speed", false);
      return;
    }
    Limelight.PoseEstimate botPose = Limelight.getBotPoseEstimate_wpiBlue(limelight);

    // Reject an empty pose.
    if (botPose.tagCount < 1) {
      return;
    }

    // Reject a pose outside of the field.
    if (!Constants.Vision.fieldBoundary.isPoseWithinArea(botPose.pose)) {
      return;
    }
    // Reject pose from long disance or high ambiguity.
    if ((botPose.tagCount == 1
            && (botPose.avgTagDist > Constants.Vision.maxSingleTagDistanceToAccept
                /*|| botPose.rawFiducials[0].ambiguity >= 0.9*/))
        || (botPose.tagCount >= 2
            && botPose.avgTagDist > Constants.Vision.maxMutiTagDistToAccept)) {
      return;
    }
    // Trust close multi tag pose when disabled with increased confidence.
    if (Math.abs(currentSpeed.vxMetersPerSecond) < 0.1
        && Math.abs(currentSpeed.vyMetersPerSecond) < 0.1
        && Math.abs(Math.toDegrees(currentSpeed.omegaRadiansPerSecond)) < 10
        && botPose.tagCount >= 2
        && botPose.avgTagDist < Constants.Vision.maxTagDistToTrust) {
      DRIVETRAIN.addVisionMeasurement(
          botPose.pose, botPose.timestampSeconds, Constants.Vision.absoluteTrustVector);
      return;
    }
    final double botPoseToPoseDistance =
        botPose.pose.getTranslation().getDistance(DRIVETRAIN.getPose().getTranslation());
    // Reject a pose that is far away from the current robot pose.
    if (botPoseToPoseDistance > 0.5) {
      System.out.println("Rejecting, Bot pose and limelight pose to far");
      return;
    }
    double tagDistanceFeet = Units.metersToFeet(botPose.avgTagDist);
    // Have a lower confidence with single tag pose proportionate to distance.
    if (botPose.tagCount == 1) {
      tagDistanceFeet *= 2;
    }
    double confidence = 0.7 + (tagDistanceFeet / 100);
    DriverStation.reportWarning("New pose from vision: " + botPose.pose.toString(), false);
    DRIVETRAIN.addVisionMeasurement(
        botPose.pose, botPose.timestampSeconds, VecBuilder.fill(confidence, confidence, 99));
  }

  public static RobotContainer get() {
    return instance;
  }
}
