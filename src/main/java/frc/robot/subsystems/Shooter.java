// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {

  private final TalonFX SHOOTER_LEADER;
  private final TalonFX SHOOTER_FOLLOWER;
  private final TalonFX FEEDER;

  private final VelocityVoltage VOLTAGE_VELOCITY_LEADER;
  private final VelocityVoltage VOLTAGE_VELOCITY_FOLLOWER;
  private final double SHOOTER_CONFIG_FEEDFOWARD = 0.15;

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

    SHOOTER_CONFIG.Slot0.kP = 0.75;
    SHOOTER_CONFIG.Slot0.kA = 0.74; // acceleration
    SHOOTER_CONFIG.Slot0.kI =
        0.0; // An error of 1 rotation per second increases output by 0.5V every second
    SHOOTER_CONFIG.Slot0.kD =
        0.000; // A change of 1 rotation per second squared results in 0.01 volts output
    SHOOTER_CONFIG.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second
    SHOOTER_CONFIG.Slot0.kS = 0.05; // Add 0.05 V output to overcome static friction
    // Peak output of 10 volts
    SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimit = 40;
    SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    SHOOTER_CONFIG.Voltage.PeakForwardVoltage = 10;
    SHOOTER_CONFIG.Voltage.PeakReverseVoltage = -10;

    final TalonFXConfiguration FEEDER_CFG = new TalonFXConfiguration();
    FEEDER_CFG.Slot0.kP = 0.15;
    FEEDER_CFG.Slot0.kI = 0;
    FEEDER_CFG.Slot0.kD = 0;
    FEEDER_CFG.Slot0.kV = 0;
    // FEEDER_CFG.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.6;
    FEEDER_CFG.CurrentLimits.StatorCurrentLimit = 45;
    FEEDER_CFG.CurrentLimits.StatorCurrentLimitEnable = true;

    VOLTAGE_VELOCITY_LEADER =
        new VelocityVoltage(0, 0, true, SHOOTER_CONFIG_FEEDFOWARD, 0, false, false, false)
            .withSlot(0);
    VOLTAGE_VELOCITY_FOLLOWER =
        new VelocityVoltage(0, 0, true, SHOOTER_CONFIG_FEEDFOWARD, 0, false, false, false)
            .withSlot(0);

    SHOOTER_LEADER.getConfigurator().apply(SHOOTER_CONFIG);
    SHOOTER_FOLLOWER.getConfigurator().apply(SHOOTER_CONFIG);

    // SHOOTER_FOLLOWER.setControl(new Follower(SHOOTER_LEADER.getDeviceID(), true));
    SHOOTER_LEADER.setInverted(true);
    FEEDER.getConfigurator().apply(FEEDER_CFG);
    FEEDER.setInverted(true);
    FEEDER.setNeutralMode(NeutralModeValue.Brake);

    // setupShuffleboard();
  }

  public double getError() {
    return SHOOTER_LEADER.getClosedLoopError().getValueAsDouble();
  }

  public void lockPositionFeeder() {
    FEEDER.setPosition(0.0);
    FEEDER.setControl(new PositionVoltage(FEEDER.getPosition().getValueAsDouble()));
  }

  public void setRPMShoot(double RPM) {
    SHOOTER_LEADER.setControl(VOLTAGE_VELOCITY_LEADER.withVelocity((RPM) / 60.0));
    SHOOTER_FOLLOWER.setControl(
        VOLTAGE_VELOCITY_FOLLOWER.withVelocity((RPM * Constants.Shooter.SPIN_RATIO) / 60.0));
  }

  public void setRPMShootAntiSpin(double RPM) {
    SHOOTER_LEADER.setControl(VOLTAGE_VELOCITY_LEADER.withVelocity((RPM) / 60.0));
    SHOOTER_FOLLOWER.setControl(
        VOLTAGE_VELOCITY_FOLLOWER.withVelocity((RPM * Constants.Shooter.ANTI_SPIN_RATIO) / 60.0));
  }

  public void setRPMShootNoSpin(double RPM) {
    SHOOTER_LEADER.setControl(VOLTAGE_VELOCITY_LEADER.withVelocity(RPM / 60.0));
    SHOOTER_FOLLOWER.setControl(VOLTAGE_VELOCITY_FOLLOWER.withVelocity(RPM / 60.0));
  }

  public void setShooterVoltage(double volts) {
    SHOOTER_LEADER.setVoltage(volts);
    SHOOTER_FOLLOWER.setVoltage(volts);
  }

  public boolean isCenterBroken() {
    return SHOOTER_FOLLOWER.getForwardLimit().asSupplier().get().value == 0;
  }

  public boolean isRearBroken() {
    return SHOOTER_FOLLOWER.getReverseLimit().asSupplier().get().value == 0;
  }

  public void setFeederVoltage(double voltage) {
    FEEDER.setVoltage(voltage);
    // FEEDER_TEMP.setVoltage(voltage);
  }

  public boolean isFeeding() {
    return Math.abs(FEEDER.getVelocity().getValueAsDouble()) > 4;
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
    return SHOOTER_LEADER.getClosedLoopError().getValueAsDouble() < 0.8;
  }

  public void setupShuffleboard() {
    SHOOTER_TAB.addDouble("Lead RPM", this::getRPMLeader);
    SHOOTER_TAB.addDouble("ERROR", this::getError);
    SHOOTER_TAB.addBoolean("Center Beam Break", this::isCenterBroken);
    SHOOTER_TAB.addBoolean("Rear Beam Break", this::isRearBroken);
    SHOOTER_TAB.add("Stop Shoot", new InstantCommand(this::stopShooter));
    SHOOTER_TAB.add("Stop Feed", new InstantCommand(this::stopFeeder));
    SHOOTER_TAB.add(
        "Reverse Feeder",
        new InstantCommand(() -> setFeederVoltage(-10.0), this)
            .andThen(new WaitCommand(2.0))
            .andThen(this::stopFeeder, this));
  }
}
