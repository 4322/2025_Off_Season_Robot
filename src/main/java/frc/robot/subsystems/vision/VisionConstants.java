package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

  // Camera names, must match names configured on coprocessor
  public static String leftCamName = "left";
  public static String rightCamName = "right";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d leftCameraTransform =
      new Transform3d(
          0.22586, 0.16431, 0.24725, new Rotation3d(0.0, Units.degreesToRadians(-25), 0.0));
  public static Transform3d rightCameraTransform =
      new Transform3d(
          0.22586, -0.16431, 0.24725, new Rotation3d(0.0, Units.degreesToRadians(-25), 0.0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.15;
  public static double maxZError = 0.75;

  public static double stdDevBaseline = 0.2;
  public static double thetaStdDevBaseline = 0.075;
}
