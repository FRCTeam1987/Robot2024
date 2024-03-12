// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class VisionConstants {
  public static final String CAMERA_NAME_SPEAKER = "Arducam_OV2311_USB_Camera_1";
  public static final String CAMERA_NAME_AMP = "Arducam_OV2311_USB_Camera";

  public static final Transform3d ROBOT_TO_CAM_SPEAKER =
      new Transform3d(
          new Translation3d(-0.185, 0, 0.3),
          new Rotation3d(0, Math.toRadians(0.0), Math.toRadians(180.0))
              .rotateBy(new Rotation3d(0, 40.0, 0)));
  public static final Transform3d ROBOT_TO_CAM_AMP =
      new Transform3d(new Translation3d(-0.156, 0, 0.354), new Rotation3d(0, 50.0, 0));

  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}
