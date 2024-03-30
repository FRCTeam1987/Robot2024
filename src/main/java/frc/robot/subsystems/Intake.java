package frc.robot.subsystems;

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

public class Intake extends SubsystemBase {
  private final TalonFX INTAKE_TOP;
  private final TalonFX INTAKE_BOTTOM;
  private final ShuffleboardTab INTAKE_TAB = Shuffleboard.getTab("INTAKE");

  public Intake(final int INTAKE_OUT_ID, final int INTAKE_IN_ID) {
    INTAKE_TOP = new TalonFX(INTAKE_OUT_ID, "canfd");
    INTAKE_BOTTOM = new TalonFX(INTAKE_IN_ID, "canfd");

    final Slot0Configs TOP_SLOT0_CFG = new Slot0Configs();
    TOP_SLOT0_CFG.kP = 0.4;
    TOP_SLOT0_CFG.kI = 0.6;
    TOP_SLOT0_CFG.kD = 0;
    TOP_SLOT0_CFG.kV = 0.01;

    final Slot0Configs BOTTOM_SLOT0_CFG = new Slot0Configs();
    BOTTOM_SLOT0_CFG.kP = 0.4;
    BOTTOM_SLOT0_CFG.kI = 0.6;
    BOTTOM_SLOT0_CFG.kD = 0;
    BOTTOM_SLOT0_CFG.kV = 0.01;

    final TalonFXConfiguration TOP_CFG = new TalonFXConfiguration();
    TOP_CFG.Slot0 = TOP_SLOT0_CFG;
    TOP_CFG.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.6;
    TOP_CFG.CurrentLimits.StatorCurrentLimit = 30;
    TOP_CFG.CurrentLimits.StatorCurrentLimitEnable = true;

    final TalonFXConfiguration BOTTOM_CFG = new TalonFXConfiguration();
    BOTTOM_CFG.Slot0 = BOTTOM_SLOT0_CFG;
    BOTTOM_CFG.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.6;
    BOTTOM_CFG.CurrentLimits.StatorCurrentLimit = 30;
    BOTTOM_CFG.CurrentLimits.StatorCurrentLimitEnable = true;

    INTAKE_TOP.getConfigurator().apply(TOP_CFG);
    INTAKE_BOTTOM.getConfigurator().apply(BOTTOM_CFG);

    INTAKE_BOTTOM.setInverted(false);
    INTAKE_TOP.setNeutralMode(NeutralModeValue.Coast);
    INTAKE_BOTTOM.setNeutralMode(NeutralModeValue.Coast);
    // setupShuffleboard();
  }

  public void setRPM(double RPM) {
    // CANNOT FOLLOW. DIFFERENT PID CTRL FOR EACH ROLLER.
    VelocityVoltage ctl = new VelocityVoltage(0);
    INTAKE_BOTTOM.setControl(ctl.withVelocity(RPM / 60.0));
    INTAKE_TOP.setControl(ctl.withVelocity(RPM / 60.0));
  }

  public void setVolts(double VOLTS) {
    // CANNOT FOLLOW. DIFFERENT PID CTRL FOR EACH ROLLER.
    INTAKE_TOP.setVoltage(VOLTS);
    INTAKE_BOTTOM.setVoltage(VOLTS);
  }

  public void stopCollecting() {
    INTAKE_TOP.set(0.0);
    INTAKE_BOTTOM.set(0.0);
  }

  public void stopTop() {
    INTAKE_TOP.set(0.0);
  }

  public double getRPMTop() {
    return INTAKE_BOTTOM.getVelocity().getValueAsDouble() * 60;
  }

  public double getAmps() {
    return INTAKE_TOP.getStatorCurrent().getValueAsDouble();
  }

  public double getRPMBottom() {
    return INTAKE_TOP.getVelocity().getValueAsDouble() * 60;
  }

  public void setupShuffleboard() {
    INTAKE_TAB.addDouble("Current Top", this::getAmps);
    INTAKE_TAB.addDouble("RL-RPM Bot", this::getRPMBottom);
    GenericEntry customRPMIn = INTAKE_TAB.add("Desired RPM", 900).getEntry();
    INTAKE_TAB.add("Start", new InstantCommand(() -> setRPM(customRPMIn.getDouble(900))));
    INTAKE_TAB.add("Stop", new InstantCommand(this::stopCollecting));
  }
}
