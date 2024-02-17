package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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

    final Slot0Configs INTAKE_TOP_CFG = new Slot0Configs();
    INTAKE_TOP_CFG.kP = 0.1;
    INTAKE_TOP_CFG.kI = 0;
    INTAKE_TOP_CFG.kD = 0;
    INTAKE_TOP_CFG.kV = 0.01;

    final Slot0Configs INTAKE_BOTTOM_CFG = new Slot0Configs();
    INTAKE_BOTTOM_CFG.kP = 0.1;
    INTAKE_BOTTOM_CFG.kI = 0;
    INTAKE_BOTTOM_CFG.kD = 0;
    INTAKE_BOTTOM_CFG.kV = 0.01;

    INTAKE_TOP.getConfigurator().apply(INTAKE_BOTTOM_CFG);
    INTAKE_BOTTOM.getConfigurator().apply(INTAKE_TOP_CFG);

    INTAKE_TOP.setInverted(true);
    setupShuffleboard();
  }

  public void setRPM(double RPM) {
    // CANNOT FOLLOW. DIFFERENT PID CTRL FOR EACH ROLLER.
    VelocityVoltage bottomCtl = new VelocityVoltage(0);
    INTAKE_BOTTOM.setControl(bottomCtl.withVelocity(((RPM / 100) * 2048) / 600));
    VelocityVoltage topCtl = new VelocityVoltage(0);
    INTAKE_TOP.setControl(topCtl.withVelocity(((RPM / 100) * 2048) / 600));
  }

  public void stopCollecting() {
    INTAKE_TOP.set(0.0);
    INTAKE_BOTTOM.set(0.0);
  }

  public double getRPMTop() {
    return INTAKE_BOTTOM.getVelocity().getValueAsDouble() * 60;
  }

  public double getRPMBottom() {
    return INTAKE_TOP.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public void periodic() {}

  public void setupShuffleboard() {
    INTAKE_TAB.addDouble("RL-RPM Top", () -> getRPMTop());
    INTAKE_TAB.addDouble("RL-RPM Bot", () -> getRPMBottom());
    GenericEntry customRPMIn = INTAKE_TAB.add("Desired RPM", 900).getEntry();
    INTAKE_TAB.add("Start", new InstantCommand(() -> setRPM(customRPMIn.getDouble(900))));
    INTAKE_TAB.add("Stop", new InstantCommand(() -> stopCollecting()));
  }
}
