// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
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

    //jank af code to differentate between having a can bus and having a canivore
    if (CANBus.getStatus("canfd").Status == StatusCode.InvalidNetwork) {
      SHOOTER_BIG = new TalonFX(shooterBottomId);
      SHOOTER_SMALL = new TalonFX(shootereHigherId);
    } else {
      SHOOTER_BIG = new TalonFX(shooterBottomId, "canfd");
      SHOOTER_SMALL = new TalonFX(shootereHigherId, "canfd");
    }
    setupShuffleboard();
    SHOOTER_SMALL.setInverted(true);
  }

  public void setSpeed(double speedPercent) {
    SHOOTER_BIG.set(speedPercent);
    SHOOTER_SMALL.set(speedPercent);
  }

  public void setSpeed(double higherSpeedPercent, double bottomSpeedPercent) {
    SHOOTER_SMALL.set(higherSpeedPercent);
    SHOOTER_BIG.set(bottomSpeedPercent);
  }

  public double getPosition() {
    return SHOOTER_BIG.getRotorPosition().getValueAsDouble();
  }

  public void stop() {
    SHOOTER_BIG.set(0.0);
    SHOOTER_SMALL.set(0.0);
  }

  public Supplier<Double> getRPMBig() {
    return SHOOTER_BIG.getVelocity().asSupplier();
  }

  public Supplier<Double> getRPMSmall() {
    return SHOOTER_SMALL.getVelocity().asSupplier();
  }

  public void setLogRPM(boolean setShouldLogRPM) {
    shouldLogRPM = setShouldLogRPM;
  } 

  public void setLogRPM() {
    shouldLogRPM = !shouldLogRPM;
  }

  public void setupShuffleboard() {

    // double[] speeds = new double[]{0.1, 0.5, 0.75, 0.8, 0.9};
    // for (double speeds2 : speeds) {
    //   SHOOTER_TAB.add("Start " + (speeds2 * 100) + "%", new InstantCommand(() -> {setSpeed(speeds2);}));
    // }

    SHOOTER_TAB.addDouble("RPM Big", () -> getRPMBig().get() * 60);
    SHOOTER_TAB.addDouble("RPM Small", () -> getRPMSmall().get() * 60);
    SHOOTER_TAB.addBoolean("INV Big", () -> SHOOTER_BIG.getInverted());
    SHOOTER_TAB.addBoolean("INV Small", () -> SHOOTER_SMALL.getInverted());
    SHOOTER_TAB.add("Invert Big", new InstantCommand(() -> invertTalon(SHOOTER_BIG)));
    SHOOTER_TAB.add("Invert Small", new InstantCommand(() -> invertTalon(SHOOTER_SMALL)));

    GenericEntry customSpeedEntry = SHOOTER_TAB.add("Custom Spd ALL", 0.1).getEntry();
    GenericEntry customSmallSpeedEntry = SHOOTER_TAB.add("Custom Spd Small", 0.1).getEntry();
    GenericEntry customBigSpeedEntry = SHOOTER_TAB.add("Custom Spd Big", 0.1).getEntry();
  
    SHOOTER_TAB.add("Run Custom Spd All", new InstantCommand(() -> setSpeed(customSpeedEntry.getDouble(0.1))));
    SHOOTER_TAB.add("Run Custom Spd Diff", new InstantCommand(() -> setSpeed(customSmallSpeedEntry.getDouble(0.1), customBigSpeedEntry.getDouble(0.1))));
    SHOOTER_TAB.add("Stop", new InstantCommand(() -> stop()));

    //SHOOTER_TAB.add("Should Log RPM", new InstantCommand(() -> setLogRPM()));
  }

  public void invertTalon(TalonFX talon) {
    talon.setInverted(!talon.getInverted());
  }
  @Override
  public void periodic() {
    //System.out.println(LimelightHelpers.getLimelightNTDouble("limelight-scoring", "ts"));
    if (shouldLogRPM == true) {
      System.out.println("Big RPM: " + getRPMBig().get() * 60);
      System.out.println("Small RPM: " + getRPMSmall().get() * 60);
    }
    // This method will be called once per scheduler run
  }
}
