// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {

  private final TalonFX SHOOTER_LEADER;
  private final TalonFX SHOOTER_FOLLOWER;
  private final TalonFX FEEDER;
  // private final CANSparkMax FEEDER_TEMP;
  private final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("SHOOTER");

  public Shooter(final int SHOOTER_LEAEDER_ID, final int SHOOTER_FOLLOWER_ID, final int FEEDER_ID) {

    SHOOTER_LEADER = new TalonFX(SHOOTER_LEAEDER_ID, "rio");
    SHOOTER_FOLLOWER = new TalonFX(SHOOTER_FOLLOWER_ID, "rio");
    FEEDER = new TalonFX(FEEDER_ID, "rio");
    // FEEDER_TEMP = new CANSparkMax(Constants.SHOOTER_FEEDER_ID_TEMP, MotorType.kBrushless);
    final TalonFXConfiguration SHOOTER_CONFIG = new TalonFXConfiguration();
    SHOOTER_CONFIG.HardwareLimitSwitch.ForwardLimitEnable = false;
    SHOOTER_CONFIG.HardwareLimitSwitch.ReverseLimitEnable = false;
    SHOOTER_CONFIG.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
    // SHOOTER_CONFIG.Slot0.kS = 0.22;
    // SHOOTER_CONFIG.Slot0.kV = 0.12;

    SHOOTER_CONFIG.Slot0.kP = 1.7;
    SHOOTER_CONFIG.Slot0.kI = 0.1;
    SHOOTER_CONFIG.Slot0.kD = 0;
    SHOOTER_CONFIG.Slot0.kV = 0.0;

    final TalonFXConfiguration FEEDER_CFG = new TalonFXConfiguration();
    FEEDER_CFG.Slot0.kP = 86.0;
    FEEDER_CFG.Slot0.kI = 1;
    FEEDER_CFG.Slot0.kD = 0;
    FEEDER_CFG.Slot0.kV = 0;
    //FEEDER_CFG.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.6;
    FEEDER_CFG.CurrentLimits.StatorCurrentLimit = 35;
    FEEDER_CFG.CurrentLimits.StatorCurrentLimitEnable = true;

    SHOOTER_LEADER.getConfigurator().apply(SHOOTER_CONFIG);
    SHOOTER_FOLLOWER.getConfigurator().apply(SHOOTER_CONFIG);

    // SHOOTER_FOLLOWER.setControl(new Follower(SHOOTER_LEADER.getDeviceID(), true));
    SHOOTER_FOLLOWER.setInverted(true);
    FEEDER.getConfigurator().apply(FEEDER_CFG);
    FEEDER.setInverted(true);
    FEEDER.setNeutralMode(NeutralModeValue.Brake);

    // FEEDER_TEMP.setInverted(true);
    // FEEDER_TEMP.setIdleMode(IdleMode.kBrake);

    setupShuffleboard();
  }

  public void setRPMShoot(double RPM) {
    VelocityVoltage ctrlLeader = new VelocityVoltage(0);
    VelocityVoltage ctrlFollower = new VelocityVoltage(0);
    SHOOTER_LEADER.setControl(ctrlLeader.withVelocity(RPM / 60.0));
    SHOOTER_FOLLOWER.setControl(ctrlFollower.withVelocity((RPM * Constants.SPIN_RATIO) / 60.0));
  }

  public boolean isLineBreakBroken() {
    return SHOOTER_LEADER.getForwardLimit().asSupplier().get().value == 0;
  }

  public void setFeederVoltage(double voltage) {
    FEEDER.setVoltage(voltage);
    // FEEDER_TEMP.setVoltage(voltage);
  }

  public void stopShooter() {
    SHOOTER_LEADER.set(0.0);
    SHOOTER_FOLLOWER.set(0.0);
  }

  public void stopFeeder() {
    // FEEDER_TEMP.set(0.0);
    FEEDER.set(0.0);
  }

  public double getRPMLeader() {
    return SHOOTER_LEADER.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getRPMFollower() {
    return SHOOTER_FOLLOWER.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getRPMFeeder() {
    return FEEDER.getVelocity().getValueAsDouble() * 60;
    // return 0.0;
  }

  public double getFeederCurrent() {
    // return FEEDER_TEMP.getOutputCurrent();
    return FEEDER.getStatorCurrent().getValueAsDouble();
  }

  public boolean isShooterAtSetpoint() {
    return SHOOTER_LEADER.getClosedLoopError().getValueAsDouble() < 10;
  }

  public void setupShuffleboard() {

    SHOOTER_TAB.addDouble("Follow RPM", () -> getRPMFollower());
    SHOOTER_TAB.addDouble("Lead RPM", () -> getRPMLeader());
    SHOOTER_TAB.addDouble("Feeder Current", () -> this.getFeederCurrent());
    // SHOOTER_TAB.addDouble("Lead RPM", this::getRPMLeader);
     SHOOTER_TAB.addDouble("SHT Err", () ->
     SHOOTER_LEADER.getClosedLoopError().getValueAsDouble());
    // SHOOTER_TAB.addDouble("FD Vlts", () -> FEEDER_TEMP.getBusVoltage());
    // SHOOTER_TAB.addDouble("FD RPM", () -> getRPMFeeder());

    SHOOTER_TAB.addBoolean("HasNote", () -> isLineBreakBroken());

    GenericEntry feedRPM = SHOOTER_TAB.add("Desired FD Vlts", 900).getEntry();
    GenericEntry shootRPM = SHOOTER_TAB.add("Desired SHT RPM", 900).getEntry();

    SHOOTER_TAB.add(
        "Run FD Vlts", new InstantCommand(() -> setFeederVoltage(feedRPM.getDouble(0))));
    SHOOTER_TAB.add("Run SHT RPM", new InstantCommand(() -> setRPMShoot(shootRPM.getDouble(0))));
    SHOOTER_TAB.add("Stop SHT", new InstantCommand(() -> stopShooter()));
    SHOOTER_TAB.add("Stop FD", new InstantCommand(() -> stopFeeder()));
    SHOOTER_TAB.add(
        "Reverse Feeder",
        new InstantCommand(() -> setFeederVoltage(-10.0), this)
            .andThen(new WaitCommand(2.0))
            .andThen(() -> stopFeeder(), this));
  }

  @Override
  public void periodic() {}

  public double ShooterCameraDistanceToTarget(double targetHeight) {
    return LimelightHelpers.calculateDistanceToTarget(
        LimelightHelpers.getTY(Constants.LIMELIGHT_SCORING),
        Constants.SHOOTER_LIMELIGHT_HEIGHT,
        targetHeight,
        Constants.SHOOTER_LIMELIGHT_ANGLE);
  }
}
