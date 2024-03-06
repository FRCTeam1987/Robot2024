// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  private double IncrementValue = 0.0;

  public Elevator(final int ELEVATOR_LEADER_ID, final int ELEVATOR_FOLLOWER_ID) {
    ELEVATOR_LEADER = new TalonFX(ELEVATOR_LEADER_ID, "canfd");
    ELEVATOR_FOLLOWER = new TalonFX(ELEVATOR_FOLLOWER_ID, "canfd");

    TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
    extensionConfig.Slot0.kP = ElevatorConstants.EXTENSION_KP;
    extensionConfig.Slot0.kI = ElevatorConstants.EXTENSION_KI;
    extensionConfig.Slot0.kD = ElevatorConstants.EXTENSION_KD;
    extensionConfig.Slot0.kV = ElevatorConstants.EXTENSION_KV;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.EXTENSION_CURRENT_LIMIT;
    extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.Feedback.RotorToSensorRatio = -1;

    extensionConfig.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.EXTENSION_MOTION_ACCELERATION;
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.EXTENSION_CRUISE_VELOCITY;
    // extensionConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.EXTENSION_JERK;

    ELEVATOR_LEADER.getConfigurator().apply(extensionConfig);
    ELEVATOR_FOLLOWER.getConfigurator().apply(extensionConfig);

    ELEVATOR_LEADER.setPosition(0);

    ELEVATOR_FOLLOWER.setControl(new Follower(ELEVATOR_LEADER.getDeviceID(), false));

    setupShuffleboard();
  }

  public void setLengthInches(double LENGTH) {
    LENGTH = LENGTH + IncrementValue;
    if (LENGTH > ElevatorConstants.MAXIMUM_EXTENSION_LENGTH_INCHES
        || LENGTH < ElevatorConstants.MINIMUM_EXTENSION_LENGTH_INCHES) {
      DriverStation.reportError("Attempt to raise elevator beyond maximum height!", false);
      return;
    } else {
      MotionMagicVoltage ctrl = new MotionMagicVoltage(0);
      ELEVATOR_LEADER.setControl(
          ctrl.withPosition(LENGTH * ElevatorConstants.CONVERSION_FACTOR_INCHES_TO_TICKS));
    }
  }

  public void incrementElevator(double IncrementAmount) {
    IncrementValue = IncrementValue + IncrementAmount;
    System.out.println("Elevator Increment Value now: " + IncrementValue);
  }

  public double getIncrementValue() {
    return IncrementValue;
  }

  public double getLengthInches() {
    return ELEVATOR_LEADER.getRotorPosition().getValueAsDouble()
        * ElevatorConstants.CONVERSION_FACTOR_TICKS_TO_INCHES;
  }

  public void goHome() {
    setLengthInches(0.0);
  }

  public void zeroPosition() {
    ELEVATOR_LEADER.setPosition(0);
  }

  public void zeroVoltage() {
    MotionMagicVoltage ctrl = new MotionMagicVoltage(0);
    ELEVATOR_LEADER.setControl(ctrl);
  }

  public void coastElevator() {
    ELEVATOR_LEADER.setNeutralMode(NeutralModeValue.Coast);
    ELEVATOR_FOLLOWER.setNeutralMode(NeutralModeValue.Coast);
  }

  public void brakeElevator() {
    ELEVATOR_LEADER.setNeutralMode(NeutralModeValue.Brake);
    ELEVATOR_FOLLOWER.setNeutralMode(NeutralModeValue.Brake);
  }

  public boolean isAtSetpoint() {
    return ELEVATOR_LEADER.getClosedLoopError().getValueAsDouble()
        < ElevatorConstants.EXTENSION_ALLOWABLE_ERROR;
  }

  @Override
  public void periodic() {}

  public void setupShuffleboard() {
    GenericEntry length = ELEVATOR_TAB.add("DesiredLen In.", 10).getEntry();
    ELEVATOR_TAB.add(
        "GoTo DesiredLen", new InstantCommand(() -> setLengthInches(length.getDouble(0))));
    ELEVATOR_TAB.addDouble("ActualLen In.", () -> getLengthInches());
    ELEVATOR_TAB.add("Coast", new InstantCommand(() -> coastElevator()).ignoringDisable(true));
    ELEVATOR_TAB.add("Brake", new InstantCommand(() -> brakeElevator()).ignoringDisable(true));
  }
}
