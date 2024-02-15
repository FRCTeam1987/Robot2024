// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final TalonFX SHOOTER_BIG_LEADER;
  private final TalonFX SHOOTER_BIG_FOLLOWER;
  private final TalonFX SHOOTER_FEEDER;
  private final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("SHOOTER");

  public Shooter(int shooterBigLEADERId, int shooterBigFollowerId, int shootereSmallLEADERId) {

    SHOOTER_BIG_LEADER = new TalonFX(shooterBigLEADERId, "rio");
    SHOOTER_BIG_FOLLOWER = new TalonFX(shooterBigFollowerId, "rio");
    SHOOTER_FEEDER = new TalonFX(shootereSmallLEADERId, "rio");

    Slot0Configs bigCfg = new Slot0Configs();
    bigCfg.kP = 0.3;
    bigCfg.kI = 0.6;
    bigCfg.kD = 0;
    bigCfg.kV = 0.1;

    Slot0Configs feederCfg = new Slot0Configs();
    feederCfg.kP = 4.0;
    feederCfg.kI = 1;
    feederCfg.kD = 0;
    feederCfg.kV = 0;
    SHOOTER_BIG_LEADER.getConfigurator().apply(bigCfg);
    SHOOTER_BIG_FOLLOWER.getConfigurator().apply(bigCfg);
    SHOOTER_BIG_FOLLOWER.setControl(new Follower(SHOOTER_BIG_LEADER.getDeviceID(), true));

    SHOOTER_FEEDER.getConfigurator().apply(feederCfg);
    SHOOTER_FEEDER.setInverted(true);
    setupShuffleboard();
  }

  public void setRPMShoot(double RPM) {
    VelocityVoltage ctrl = new VelocityVoltage(0);
    // 2048 encoder ticks, 600 div/s talon cares about rps for some reason??
    SHOOTER_BIG_LEADER.setControl(ctrl.withVelocity(((RPM / 100) * 2048) / 600));
  }

  public boolean isLineBreakBroken() {
    return SHOOTER_BIG_LEADER.getForwardLimit().asSupplier().get().value == 0;
  }

  public void setRPMFeeder(double RPM) {
    VelocityVoltage ctrl = new VelocityVoltage(0);
    // 2048 encoder ticks, 600 div/s talon cares about rps for some reason??
    SHOOTER_FEEDER.setControl(ctrl.withVelocity(((RPM / 100) * 2048) / 600));
  }
  public void setRPMFeederInverted(double RPM) {
    VelocityVoltage ctrl = new VelocityVoltage(0);
    // 2048 encoder ticks, 600 div/s talon cares about rps for some reason??
    SHOOTER_FEEDER.setControl(ctrl.withVelocity(-(((RPM / 100) * 2048) / 600)));
  }

  public void setFeederVoltage(double voltage) {
    SHOOTER_FEEDER.setVoltage(voltage);
  }

  public void stopShooter() {
    SHOOTER_BIG_LEADER.set(0.0);
  }

  public void stopFeeder() {
    SHOOTER_FEEDER.set(0.0);
  }

  public double getRPMBig() {
    return SHOOTER_BIG_LEADER.getVelocity().getValueAsDouble() * 60;
  }

  public double getRPMSmall() {
    return SHOOTER_FEEDER.getVelocity().getValueAsDouble() * 60;
  }

  public void setupShuffleboard() {

    SHOOTER_TAB.addDouble("RL-RPM Big", () -> getRPMBig());
    SHOOTER_TAB.addDouble("RL-VLTG", () -> SHOOTER_FEEDER.getMotorVoltage().getValueAsDouble());
    SHOOTER_TAB.addDouble("RL-RPM Feed", () -> getRPMSmall());

    SHOOTER_TAB.addBoolean("LineBreak", () -> isLineBreakBroken());

    GenericEntry feedRPM = SHOOTER_TAB.add("RPM Feed", 900).getEntry();
    GenericEntry bigRPM = SHOOTER_TAB.add("RPM Big", 900).getEntry();

    SHOOTER_TAB.add("Run Feed", new InstantCommand(() -> setRPMFeeder(feedRPM.getDouble(0))));
    SHOOTER_TAB.add("Run Shoot", new InstantCommand(() -> setRPMShoot(bigRPM.getDouble(0))));
    SHOOTER_TAB.add("ShootStop", new InstantCommand(() -> stopShooter()));
    SHOOTER_TAB.add("ShotNote", new InstantCommand(() -> isLineBreakBroken()));
    SHOOTER_TAB.add("FeedStop", new InstantCommand(() -> stopFeeder()));
    // SHOOTER_TAB.add("Run Shoot", new InstantCommand(()  ->
    // setSpeed(customSmallSpeedEntry.getDouble(0.1), 0.0))
    //   .andThen(new WaitCommand(2)
    //   .andThen(new InstantCommand(()  -> setSpeed(customSmallSpeedEntry.getDouble(0.1),
    // customBigSpeedEntry.getDouble(0.1)))
    //   .andThen(new  WaitCommand(1)
    //   .andThen(new InstantCommand(() -> stop()))))));

    // SHOOTER_TAB.add("Should Log RPM", new InstantCommand(() -> setLogRPM()));
  }

  @Override
  public void periodic() {}
}
