package org.curtinfrc.frc2026.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static String camera0Name = "front-left";
  public static String camera1Name = "front-right";
  public static String camera2Name = "back-left";
  public static String camera3Name = "back-right";

  public static Transform3d robotToCamera0 =
      new Transform3d(293.052, -289.311, 445.512, Rotation3d.kZero);
  public static Transform3d robotToCamera1 =
      new Transform3d(-293.302, -289.311, 445.512, Rotation3d.kZero);
  public static Transform3d robotToCamera2 =
      new Transform3d(292.205, -41.151, 474.879, Rotation3d.kZero);
  public static Transform3d robotToCamera3 =
      new Transform3d(-291.955, -41.151, 474.879, Rotation3d.kZero);
}
