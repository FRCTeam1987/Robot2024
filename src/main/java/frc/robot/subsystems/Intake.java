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

public class Intake extends SubsystemBase {
  private final TalonFX INTAKE_OUT;
  private final TalonFX INTAKE_IN;
  private final ShuffleboardTab INTAKE_TAB = Shuffleboard.getTab("INTAKE");

  public Intake(int INTAKE_OUT_ID, int INTAKE_IN_ID) {
    if (CANBus.getStatus("canfd").Status == StatusCode.InvalidNetwork) {
      INTAKE_OUT = new TalonFX(INTAKE_OUT_ID);
      INTAKE_IN = new TalonFX(INTAKE_IN_ID);
    } else {
      INTAKE_OUT = new TalonFX(INTAKE_OUT_ID, "canfd");
      INTAKE_IN = new TalonFX(INTAKE_IN_ID, "canfd");
    }

    Slot0Configs intakeInCfg = new Slot0Configs();
    intakeInCfg.kP = 0.1;
    intakeInCfg.kI = 0;
    intakeInCfg.kD = 0;
    intakeInCfg.kV = 0.01;

    Slot0Configs intakeOutCfg = new Slot0Configs();
    intakeOutCfg.kP = 0.1;
    intakeOutCfg.kI = 0;
    intakeOutCfg.kD = 0;
    intakeOutCfg.kV = 0.01;
    INTAKE_OUT.getConfigurator().apply(intakeOutCfg);
    INTAKE_IN.getConfigurator().apply(intakeInCfg);
    setupShuffleboard();
  }

    public void setRPMIn(double RPM) {
    VelocityVoltage ctrl = new VelocityVoltage(0);
    INTAKE_IN.setControl(ctrl.withVelocity(((RPM / 100)  *  2048) / 600));
  }

      public void setRPMOut(double RPM) {
    VelocityVoltage ctrl = new VelocityVoltage(0);
    INTAKE_OUT.setControl(ctrl.withVelocity(((RPM / 100)  *  2048) / 600));
  }

  public void stopCollecting() {
    INTAKE_OUT.set(0.0);
    INTAKE_IN.set(0.0);
  }

  public double getRPMIn() {
    return INTAKE_IN.getVelocity().getValueAsDouble() * 60;
  }

  public double getRPMOut() {
    return INTAKE_OUT.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public void periodic() {}

  public void setupShuffleboard() {
    INTAKE_TAB.addDouble("RL-RPM In", () -> getRPMIn());
    INTAKE_TAB.addDouble("RL-RPM Out", () -> getRPMOut());
    GenericEntry customRPMIn = INTAKE_TAB.add("OUT RPM", 900).getEntry();
    GenericEntry customRPMOut = INTAKE_TAB.add("IN RPM", 900).getEntry();
    INTAKE_TAB.add("Start IN", new InstantCommand(() -> setRPMIn(customRPMIn.getDouble(900))));
    INTAKE_TAB.add("Start OUT", new InstantCommand(() -> setRPMOut(customRPMOut.getDouble(900))));
    INTAKE_TAB.add("Stop", new InstantCommand(() -> stopCollecting()));
    // INTAKE_TAB.addInteger("Number of Notes", () -> numberOfNotesCollected());
  }

}