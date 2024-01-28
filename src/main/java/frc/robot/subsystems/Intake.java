package frc.robot.subsystems;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX INTAKE_OUT;
  private final TalonFX INTAKE_IN;
  private boolean isNoteCollected = false;
  private final ShuffleboardTab INTAKE_TAB = Shuffleboard.getTab("INTAKE");
  private int notesCollected = 0;
  private int isInverted = 1; //1 is not inverted -1 is inverted

  public Intake(int INTAKE_OUT_ID, int INTAKE_IN_ID) {
    if (CANBus.getStatus("canfd").Status == StatusCode.InvalidNetwork) {
      INTAKE_OUT = new TalonFX(INTAKE_OUT_ID);
      INTAKE_IN = new TalonFX(INTAKE_IN_ID);
    } else {
      INTAKE_OUT = new TalonFX(INTAKE_OUT_ID, "canfd");
      INTAKE_IN = new TalonFX(INTAKE_IN_ID, "canfd");
    }
    setupShuffleboard();
    isInverted = -1;
  }

  public void collectNote() {
    // if (!isNoteCollected) {
      INTAKE_OUT.set(-0.7);
      INTAKE_IN.set(-0.7 * isInverted);
    // }
  }

  public void collectNote(double speed) {
    // if (!isNoteCollected) {
      INTAKE_OUT.set(-speed);
      INTAKE_IN.set(-speed * isInverted);
    // }
  }

  public void stopCollecting() {
    INTAKE_OUT.set(0.0);
    INTAKE_IN.set(0.0);
  }

  @Override
  public void periodic() {}

  public boolean hasCollectedNote() {
    return isNoteCollected;
  }

  public int numberOfNotesCollected() {
    return notesCollected;
  }

  public void setNoteCollected(boolean setIsNoteCollected) { //Use to stop collecting excess notes
    isNoteCollected = setIsNoteCollected;
  } 

  public void setupShuffleboard() {
    double[] speeds = new double[]{0.1, 0.5, 0.75, 0.9};
    for (double speeds2 : speeds) {
      INTAKE_TAB.add("Start " + (speeds2 * 100) + "%", new InstantCommand(() -> {collectNote(speeds2);}));
    }

    GenericEntry customSpeedEntry = INTAKE_TAB.add("Custom Speed", 0.1).getEntry();
    INTAKE_TAB.add("Start Custom", new InstantCommand(() -> collectNote(customSpeedEntry.getDouble(0.1))));
    INTAKE_TAB.add("Stop", new InstantCommand(() -> stopCollecting()));
    // INTAKE_TAB.addInteger("Number of Notes", () -> numberOfNotesCollected());
  }

}