// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final TalonFX SHOOTER_BIG;
  private final TalonFX SHOOTER_SMALL;
  private final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("SHOOTER");
  private boolean shouldLogRPM = false;

  public Shooter(int shooterBottomId, int shootereHigherId) {

    // jank af code to differentate between having a can bus and having a canivore
    if (CANBus.getStatus("canfd").Status == StatusCode.InvalidNetwork) {
      SHOOTER_BIG = new TalonFX(shooterBottomId);
      SHOOTER_SMALL = new TalonFX(shootereHigherId);
    } else {
      SHOOTER_BIG = new TalonFX(shooterBottomId, "canfd");
      SHOOTER_SMALL = new TalonFX(shootereHigherId, "canfd");
    }

    Slot0Configs smallCfg = new Slot0Configs();
    smallCfg.kP = 0.12;
    smallCfg.kI = 0;
    smallCfg.kD = 0;
    smallCfg.kV = 0.01;

    Slot0Configs bigCfg = new Slot0Configs();
    bigCfg.kP = 0.12;
    bigCfg.kI = 0;
    bigCfg.kD = 0;
    bigCfg.kV = 0.01;
    SHOOTER_BIG.getConfigurator().apply(bigCfg);
    SHOOTER_SMALL.getConfigurator().apply(smallCfg);
    setupShuffleboard();
    SHOOTER_SMALL.setInverted(true);
  }

  public void setRPMSmall(double RPM) {
    VelocityVoltage ctrl = new VelocityVoltage(0);
    // 2048 encoder ticks, 600 div/s talon cares about rps for some reason??
    SHOOTER_BIG.setControl(ctrl.withVelocity(((RPM / 100) * 2048) / 600));
  }

  public void setRPMBig(double RPM) {
    VelocityVoltage ctrl = new VelocityVoltage(0);
    // 2048 encoder ticks, 600 div/s talon cares about rps for some reason??
    SHOOTER_SMALL.setControl(ctrl.withVelocity(((RPM / 100) * 2048) / 600));
  }

  public void stop() {
    SHOOTER_BIG.set(0.0);
    SHOOTER_SMALL.set(0.0);
  }

  public double getRPMBig() {
    return SHOOTER_BIG.getVelocity().getValueAsDouble() * 60;
  }

  public double getRPMSmall() {
    return SHOOTER_SMALL.getVelocity().getValueAsDouble() * 60;
  }

  public void setupShuffleboard() {

    SHOOTER_TAB.addDouble("RL-RPM Big", () -> getRPMBig());
    SHOOTER_TAB.addDouble("RL-RPM Small", () -> getRPMSmall());

    GenericEntry smallRPM = SHOOTER_TAB.add("RPM Small", 900).getEntry();
    GenericEntry bigRPM = SHOOTER_TAB.add("RPM Big", 900).getEntry();

    SHOOTER_TAB.add("Run Small RPM", new InstantCommand(() -> setRPMSmall(smallRPM.getDouble(0))));
    SHOOTER_TAB.add("Run Big RPM", new InstantCommand(() -> setRPMBig(bigRPM.getDouble(0))));
    SHOOTER_TAB.add("Stop", new InstantCommand(() -> stop()));
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
