// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.commands.zeroing.ZeroElevator;
import frc.robot.constants.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX ELEVATOR_LEADER;

  private final TalonFX ELEVATOR_FOLLOWER;
  private final ShuffleboardTab ELEVATOR_TAB = Shuffleboard.getTab("ELEVATOR");
  private double IncrementValue = 0.0;

  public Elevator(
      final int ELEVATOR_LEADER_ID,
      final int ELEVATOR_FOLLOWER_ID) { // TODO don't use these since they're the same as defined in
    // constants
    ELEVATOR_LEADER = new TalonFX(ELEVATOR_LEADER_ID, "canfd");
    ELEVATOR_FOLLOWER = new TalonFX(ELEVATOR_FOLLOWER_ID, "canfd");

    TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
    extensionConfig.Slot0.kP = Constants.Elevator.EXTENSION_KP;
    extensionConfig.Slot0.kI = Constants.Elevator.EXTENSION_KI;
    extensionConfig.Slot0.kD = Constants.Elevator.EXTENSION_KD;
    extensionConfig.Slot0.kV = Constants.Elevator.EXTENSION_KV;
    extensionConfig.Slot1.kP = Constants.Elevator.EXTENSION_KP_1;
    extensionConfig.Slot1.kI = Constants.Elevator.EXTENSION_KI_1;
    extensionConfig.Slot1.kD = Constants.Elevator.EXTENSION_KD_1;
    extensionConfig.Slot1.kV = Constants.Elevator.EXTENSION_KV_1;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.Elevator.EXTENSION_CURRENT_LIMIT;
    extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.Feedback.RotorToSensorRatio = -1;

    extensionConfig.MotionMagic.MotionMagicAcceleration =
        Constants.Elevator.EXTENSION_MOTION_ACCELERATION;
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity =
        Constants.Elevator.EXTENSION_CRUISE_VELOCITY;
    // extensionConfig.MotionMagic.MotionMagicJerk = Constants.Elevator.EXTENSION_JERK;

    ELEVATOR_LEADER.getConfigurator().apply(extensionConfig);
    ELEVATOR_FOLLOWER.getConfigurator().apply(extensionConfig);

    setZero();

    ELEVATOR_FOLLOWER.setControl(new Follower(ELEVATOR_LEADER.getDeviceID(), false));

    // setupShuffleboard();
  }

  public void setVoltage(double volts) {
    ELEVATOR_LEADER.setVoltage(volts);
  }

  public double getVelocity() {
    return ELEVATOR_LEADER.getVelocity().getValueAsDouble();
  }

  public void stop() {
    ELEVATOR_LEADER.setVoltage(0);
  }

  public void setLengthInchesSlot1(double LENGTH) {
    LENGTH = LENGTH + IncrementValue;
    if (LENGTH > Constants.Elevator.MAXIMUM_EXTENSION_LENGTH_INCHES
        || LENGTH < Constants.Elevator.MINIMUM_EXTENSION_LENGTH_INCHES) {
      DriverStation.reportError("Attempt to raise elevator beyond maximum height!", false);
    } else {
      MotionMagicVoltage ctrl = new MotionMagicVoltage(0, true, 0, 1, false, false, false);

      ELEVATOR_LEADER.setControl(
          ctrl.withPosition(LENGTH * Constants.Elevator.CONVERSION_FACTOR_INCHES_TO_TICKS));
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
        * Constants.Elevator.CONVERSION_FACTOR_TICKS_TO_INCHES;
  }

  public void setLengthInches(double LENGTH) {
    LENGTH = LENGTH + IncrementValue;
    if (LENGTH > Constants.Elevator.MAXIMUM_EXTENSION_LENGTH_INCHES
        || LENGTH < Constants.Elevator.MINIMUM_EXTENSION_LENGTH_INCHES) {
      DriverStation.reportError("Attempt to raise elevator beyond maximum height!", false);
    } else {
      MotionMagicVoltage ctrl = new MotionMagicVoltage(0);
      ELEVATOR_LEADER.setControl(
          ctrl.withPosition(LENGTH * Constants.Elevator.CONVERSION_FACTOR_INCHES_TO_TICKS));
    }
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
        < Constants.Elevator.EXTENSION_ALLOWABLE_ERROR;
  }

  public void setZero() {
    ELEVATOR_LEADER.setPosition(0);
  }

  public void setupShuffleboard() {
    GenericEntry length = ELEVATOR_TAB.add("DesiredLen In.", 2).getEntry();
    ELEVATOR_TAB.add("ZERO SUBSYSTEM", new ZeroElevator(this));
    ELEVATOR_TAB.add(
        "GoTo DesiredLen", new InstantCommand(() -> setLengthInches(length.getDouble(0))));
    ELEVATOR_TAB.addDouble("ActualLen In.", this::getLengthInches);
    ELEVATOR_TAB.add("Coast", new InstantCommand(this::coastElevator).ignoringDisable(true));
    ELEVATOR_TAB.add("Brake", new InstantCommand(this::brakeElevator).ignoringDisable(true));
    ELEVATOR_TAB.addDouble("ERROR", () -> ELEVATOR_LEADER.getClosedLoopError().getValueAsDouble());
    // ELEVATOR_TAB.addnumber("Elevator voltage", );
  }
}
