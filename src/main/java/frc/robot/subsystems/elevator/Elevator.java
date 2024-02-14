// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX ELEVATOR_LEADER;

  private final TalonFX ELEVATOR_FOLLOWER;
  private final ShuffleboardTab ELEVATOR_TAB = Shuffleboard.getTab("ELEVATOR");

  public Elevator(int ELEVATOR_LEADER_ID, int ELEVATOR_FOLLOWER_ID) {
    ELEVATOR_LEADER = new TalonFX(ELEVATOR_LEADER_ID, "canfd");
    ELEVATOR_FOLLOWER = new TalonFX(ELEVATOR_FOLLOWER_ID, "canfd");

    Slot0Configs extensionPID = new Slot0Configs();
    extensionPID.kP = ElevatorConstants.EXTENSION_KP;
    extensionPID.kI = ElevatorConstants.EXTENSION_KI;
    extensionPID.kD = ElevatorConstants.EXTENSION_KD;
    extensionPID.kV = ElevatorConstants.EXTENSION_KV;

    TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
    extensionConfig.Slot0 = extensionPID;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.StatorCurrentLimit = 4;

    ELEVATOR_LEADER.getConfigurator().apply(extensionConfig);
    ELEVATOR_FOLLOWER.getConfigurator().apply(extensionConfig);
    setupShuffleboard();
  }

  public void setElevatorLengthInches(final double LENGTH) {
    if (LENGTH > ElevatorConstants.MAXIMUM_EXTENSION_LENGTH_INCHES
        || LENGTH < ElevatorConstants.MINIMUM_EXTENSION_LENGTH_INCHES) {
      DriverStation.reportError("Attempt to raise elevator beyond maximum height!", false);
      return;
    }
    PositionVoltage ctrl = new PositionVoltage(0);
    ELEVATOR_LEADER.setControl(
        ctrl.withPosition(LENGTH * ElevatorConstants.CONVERSION_FACTOR_INCHES_TO_TICKS));
  }

  public double getElevatorLengthInches() {
    return ELEVATOR_LEADER.getRotorPosition().getValueAsDouble()
        * ElevatorConstants.CONVERSION_FACTOR_TICKS_TO_INCHES;
  }

  @Override
  public void periodic() {}

  public void setupShuffleboard() {
    GenericEntry length = ELEVATOR_TAB.add("Desired Len", 10).getEntry();
    ELEVATOR_TAB.add(
        "Go To Length", new InstantCommand(() -> setElevatorLengthInches(length.getDouble(0))));
    ELEVATOR_TAB.addDouble("Actual Len", () -> getElevatorLengthInches());
    // INTAKE_TAB.addInteger("Number of Notes", () -> numberOfNotesCollected());
  }
}
