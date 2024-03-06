// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private PhotonCamera camera;
  public static Vision instance;
  private double yawVal = 0;
  private double pitchVal = 0;
  private double skewVal = 0;
  private double areaVal = 0;
  private boolean hasTarget = false;
  private boolean LED_Enable = false;
  private List<Integer> validFiducials;

  // Constants such as camera and target height stored. Change per robot and goal!
  private double CAMERA_HEIGHT_METERS = Units.inchesToMeters(0.1);
  private double TARGET_HEIGHT_METERS = Units.inchesToMeters(105);
  // Angle between horizontal and the camera.
  private double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0.1);
  private String CAMERA_NAME = "";

  public Vision(
      String photonCameraName,
      double cameraHeightMeters,
      double cameraAngleDegrees,
      List<Integer> validIDs) {

    this.camera = new PhotonCamera(photonCameraName);
    this.CAMERA_NAME = photonCameraName;
    this.CAMERA_HEIGHT_METERS = cameraHeightMeters;
    this.CAMERA_PITCH_RADIANS = Units.degreesToRadians(cameraAngleDegrees);
    this.validFiducials = validIDs;
    // this.camera.setPipelineIndex(Constants.Tape01);
  }

  public static Vision getInstance() {
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = this.camera.getLatestResult();
    if (result.hasTargets()) {
      Optional<PhotonTrackedTarget> trackedTarget =
          result.getTargets().stream()
              .filter(target -> validFiducials.contains((target.getFiducialId())))
              .findFirst();
      if (!trackedTarget.isPresent()) {
        return;
      }
      int fiducialID = trackedTarget.get().getFiducialId();
      // if (this.validFiducials.contains(fiducialID)) {
      this.yawVal = trackedTarget.get().getYaw();
      this.pitchVal = trackedTarget.get().getPitch();
      this.skewVal = trackedTarget.get().getSkew();
      this.areaVal = trackedTarget.get().getArea();
      this.hasTarget = true;

      // PHOTON_TAB.addNumber("Yaw Value", () -> yawVal);
      // PHOTON_TAB.addNumber("Pitch Value", () -> pitchVal);
      // PHOTON_TAB.addNumber("Area Value", () -> areaVal);
      // PHOTON_TAB.addBoolean("LEDs OnOff", () -> this.LED_Enable);
      // }
    } else {
      this.hasTarget = false;
    }
    if (LED_Enable) {
      cameraLEDOn();
      // Set driver mode to off.
      camera.setDriverMode(false);
    } else {
      cameraLEDOff();
      // Set driver mode to on.
      camera.setDriverMode(true);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  public String getCameraName() {
    return CAMERA_NAME;
  }

  public double getCameraHeight() {
    return CAMERA_HEIGHT_METERS;
  }

  public double getCameraRadians() {
    return CAMERA_PITCH_RADIANS;
  }

  public double getCameraDegrees() {
    return Units.radiansToDegrees(CAMERA_PITCH_RADIANS);
  }

  public double getYawVal() {
    return this.yawVal;
  }

  public double getPitchVal() {
    return this.pitchVal;
  }

  public double getSkewVal() {
    return this.skewVal;
  }

  public double getAreaVal() {
    return this.areaVal;
  }

  public boolean hasTargets() {
    return this.hasTarget;
  }

  public void cameraLEDOff() {
    this.camera.setLED(VisionLEDMode.kOff);
  }

  public void cameraLEDOn() {
    // Note that this will just turn the LEDs on
    this.camera.setLED(VisionLEDMode.kOn);
  }

  public void cameraLEDBlink() {
    this.camera.setLED(VisionLEDMode.kBlink);
  }

  public void cameraLEDToggle() {
    if (LED_Enable) {
      LED_Enable = false;
    } else {
      LED_Enable = true;
    }
  }

  public void cameraLEDToggleOff() {
    LED_Enable = false;
  }

  public void cameraLEDToggleOn() {
    LED_Enable = true;
  }

  public void cameraLED() {
    // Note that this will use the value of the pipeline for the intensity
    this.camera.setLED(VisionLEDMode.kDefault);
  }

  public double getRange() {
    double range =
        PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(getPitchVal()));
    double rangeInInches = Units.metersToInches(range);

    // PHOTON_TAB.addNumber("Camera Distance", () -> rangeInInches);

    return range;
  }

  /**
   * Calculates the distance from the camera to the target based on the vertical angle and the known
   * heights of the camera and the target.
   *
   * @param cameraName The network table name of the cameraName to query.
   * @param cameraHeight In meters from ground.
   * @param targetHeight In meters from ground.
   * @param cameraAngle In degrees.
   * @return The distance to the target in inches, or a negative value if invalid.
   */
  public static double calculateDistanceToTarget(
      double ty, double cameraHeight, double targetHeight, double cameraAngle) {
    double angleToTargetDegrees = ty + cameraAngle;
    double angleToTargetRadians = Math.toRadians(angleToTargetDegrees);
    double heightDifference = targetHeight - cameraHeight;

    // Check if the height difference is valid
    if (heightDifference <= 0) {
      // Return an error code or handle this case as needed
      System.err.println("Invalid height difference for distance calculation");
      return -1;
    }

    // Calculate the distance using trigonometry
    double distance = heightDifference / Math.tan(angleToTargetRadians);

    // System.out.println("Distance to target: " + distance);
    return distance;
  }
}
