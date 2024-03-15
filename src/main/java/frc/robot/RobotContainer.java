// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.control.AimLockWrist;
import frc.robot.commands.control.Climb;
import frc.robot.commands.control.GoHome;
import frc.robot.commands.control.IdleShooter;
import frc.robot.commands.control.ReverseIntake;
import frc.robot.commands.control.ShootSubwoofer;
import frc.robot.commands.control.ShootSubwooferFlat;
import frc.robot.commands.control.ShootTall;
import frc.robot.commands.control.StopAll;
import frc.robot.commands.control.amp.FireRevAmp;
import frc.robot.commands.control.amp.PrepRevAmp;
import frc.robot.commands.control.note.IntakeNoteSequence;
import frc.robot.commands.control.note.LobNote;
import frc.robot.commands.control.note.PoopNote;
import frc.robot.commands.control.note.ShootNote;
import frc.robot.commands.control.note.ShootNoteSequence;
import frc.robot.commands.control.note.SpitNote;
import frc.robot.commands.movement.CollectNoteAuto;
import frc.robot.commands.movement.DriveToNote;
import frc.robot.commands.movement.DriveToNoteAuto;
import frc.robot.commands.movement.FastPoint;
import frc.robot.commands.movement.SquareUpToAprilTag;
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
import frc.robot.util.Util;
import java.util.Arrays;

public class RobotContainer {
  private final SendableChooser<Command> AUTO_CHOOSER = new SendableChooser<>();
  public final ShuffleboardTab COMMANDS_TAB = Shuffleboard.getTab("COMMANDS");
  public final ShuffleboardTab MATCH_TAB = Shuffleboard.getTab("MATCH");
  public final ShuffleboardTab PHOTON_TAB = Shuffleboard.getTab("PHOTON");
  public final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("SHOOTER");
  public final ShuffleboardTab PROTO_TAB = Shuffleboard.getTab("PROTO");
  public final Vision INTAKE_PHOTON = new Vision("Arducam_OV9782_USB_Camera", 0.65176, 60);
  public final Vision SPEAKER_PHOTON =
      new Vision("Arducam_OV2311_USB_Camera_1", 0.3, 40, Arrays.asList(4, 7));
  public final Vision AMP_PHOTON =
      new Vision("Arducam_OV2311_USB_Camera", 0.35, 40, Arrays.asList(5, 6));
  public final CommandSwerveDrivetrain DRIVETRAIN = DriveConstants.DriveTrain; // My drivetrain
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
  private final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(0);
  private final CommandXboxController CO_DRIVER_CONTROLLER = new CommandXboxController(1);
  public static boolean isForwardAmpPrimed = false;
  public static boolean isReverseAmpPrimed = false;
  public static boolean isClimbPrimed = false;
  private double MaxSpeed =
      DriveConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandSwerveDrivetrain drivetrain = DriveConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureNamedCommands();
    configureShuffleboard();
    configureDrivetrain();
    configureDriverController();
    configureCoDriverController();
    configureDefaultCommands();
  }

  private void configureDriverController() {
    DRIVER_CONTROLLER.b().onTrue(new ShootSubwoofer(ELEVATOR, WRIST, SHOOTER));
    // DRIVER_CONTROLLER
    //     .y()
    //     .onTrue(
    //         new ConditionalCommand(
    //             new FireFwdAmp(SHOOTER)
    //                 .andThen(new InstantCommand(() -> isForwardAmpPrimed = false)
    //                 .andThen(new GoHome(ELEVATOR, WRIST, SHOOTER, INTAKE))),
    //             new PrepFwdAmp(ELEVATOR, WRIST, SHOOTER)
    //                 .andThen(new InstantCommand(() -> isForwardAmpPrimed = true)),
    //             () -> isForwardAmpPrimed));

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
    DRIVER_CONTROLLER.back().onTrue(DRIVETRAIN.runOnce(() -> DRIVETRAIN.seedFieldRelative()));
    DRIVER_CONTROLLER.start().onTrue(new GoHome(ELEVATOR, WRIST, SHOOTER, INTAKE));
    DRIVER_CONTROLLER.x().onTrue(new PoopNote(SHOOTER, 500));
    DRIVER_CONTROLLER
        .leftBumper()
        .onTrue(
            new IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR)
                .andThen(
                    new AsyncRumble(
                        DRIVER_CONTROLLER.getHID(), RumbleType.kBothRumble, 1.0, 700L)));
    DRIVER_CONTROLLER.rightTrigger().onTrue(new FastPoint(DRIVETRAIN, SPEAKER_PHOTON));

    DRIVER_CONTROLLER
        .rightBumper()
        .onTrue(new ShootNote(SHOOTER, ELEVATOR, Constants.Shooter.SHOOTER_RPM));
    DRIVER_CONTROLLER.leftTrigger().onTrue(new LobNote(SHOOTER, WRIST, ELEVATOR));
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
    CO_DRIVER_CONTROLLER.leftBumper().onTrue(new FastPoint(DRIVETRAIN, SPEAKER_PHOTON));

    CO_DRIVER_CONTROLLER.rightTrigger().onTrue(new ShootSubwooferFlat(ELEVATOR, WRIST, SHOOTER));
    CO_DRIVER_CONTROLLER
        .a()
        .onTrue(
            new ParallelDeadlineGroup(
                new IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR),
                new DriveToNoteAuto(DRIVETRAIN, AMP_PHOTON, SHOOTER, INTAKE, WRIST, ELEVATOR)));
  }

  private void configureDrivetrain() {

    DRIVETRAIN.setDefaultCommand( // Drivetrain will execute this command periodically
        DRIVETRAIN
            .applyRequest(
                () ->
                    drive
                        .withVelocityX(
                            -DRIVER_CONTROLLER.getLeftY()
                                * DriveConstants.kSpeedAt12VoltsMps) // Drive forward with
                        // negative Y (forward)
                        .withVelocityY(
                            -DRIVER_CONTROLLER.getLeftX()
                                * DriveConstants
                                    .kSpeedAt12VoltsMps) // Drive left with negative X (left)
                        .withRotationalRate(
                            -DRIVER_CONTROLLER.getRightX()
                                * Math.PI
                                * 3.5) // Drive counterclockwise with negative X (left)
                )
            .ignoringDisable(true));

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

  public void configureShuffleboard() {
    WRIST.setupShuffleboard();
    SHOOTER.setupShuffleboard();
    INTAKE.setupShuffleboard();
    ELEVATOR.setupShuffleboard();
    PHOTON_TAB.addDouble("DISTANCE_TO_SPEAKER", () -> Util.getInterpolatedDistance(SPEAKER_PHOTON));

    COMMANDS_TAB.add("Lob Note", new LobNote(SHOOTER, WRIST, ELEVATOR));
    COMMANDS_TAB.add("Fast Point", new FastPoint(DRIVETRAIN, SPEAKER_PHOTON));
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

    PHOTON_TAB.add(
        "Square Up AprilTag",
        new SquareUpToAprilTag(DRIVETRAIN, SPEAKER_PHOTON, Constants.SPEAKER_APRILTAG_HEIGHT, 3));
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

    SHOOTER_TAB.add("Spit Note", new SpitNote(SHOOTER));

    addAuto("ampa-full");
    addAuto("sourcea-fullshoot");
    addAuto("sourcea");
    addAuto("amp_close");
    addAuto("amp_subwoofer");
    addAuto("amp_subwoofer_reversal");
    addAuto("driven_source_score");
    addAuto("heart_source_shoot");
    addAuto("heart_source_og");
    addAuto("Taxi-Amp");
    addAuto("Taxi-Source");
    AUTO_CHOOSER.addOption("Do Nothing", new InstantCommand());
    MATCH_TAB.add("Auto", AUTO_CHOOSER);
  }

  public void configureDefaultCommands() {
    WRIST.setDefaultCommand(new AimLockWrist(WRIST, SHOOTER, ELEVATOR, SPEAKER_PHOTON));
    SHOOTER.setDefaultCommand(new IdleShooter(SHOOTER));
    CANDLES.setDefaultCommand(new DefaultCANdle(CANDLES, SHOOTER, SPEAKER_PHOTON));
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand(
        "ShootNote", new ShootNoteSequence(SHOOTER, WRIST, Constants.Shooter.SHOOTER_RPM, 40));
    NamedCommands.registerCommand(
        "ShootNoteAimbot",
        new ShootNoteSequence(
            SHOOTER, WRIST, Constants.Shooter.SHOOTER_RPM, DRIVETRAIN, SPEAKER_PHOTON));
    NamedCommands.registerCommand(
        "SpinUpShooter", new InstantCommand(() -> SHOOTER.setRPMShoot(5200)));
    NamedCommands.registerCommand(
        "ShootNoteSubFar",
        new ShootNoteSequence(SHOOTER, WRIST, ELEVATOR, Constants.Shooter.SHOOTER_RPM, 36, 10));
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
              SHOOTER.setRPMShootNoSpin(4000);
              WRIST.setDegrees(25);
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
    // NamedCommands.registerCommand("ShootAmp", new ShootAmp(SHOOTER, ELEVATOR, WRIST));
    NamedCommands.registerCommand(
        "DriveToNoteAuto",
        new ParallelDeadlineGroup(
            new IntakeNoteSequence(SHOOTER, INTAKE, WRIST, ELEVATOR),
            new DriveToNoteAuto(DRIVETRAIN, AMP_PHOTON, SHOOTER, INTAKE, WRIST, ELEVATOR)));
  }

  public void addAuto(String autoName) {
    AUTO_CHOOSER.addOption(autoName, new PathPlannerAuto(autoName));
  }

  public Command getAutonomousCommand() {
    return AUTO_CHOOSER.getSelected();
  }
}
