package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public GlobalPoseObservation[] globalPoseObservations = new GlobalPoseObservation[0];
    public SingleTagPoseObservation[] singleTagPoseObservations = new SingleTagPoseObservation[0];
    public ObservationMode observationMode = ObservationMode.GLOBAL_POSE;
    public SingleTagCamera singleTagCamToUse = SingleTagCamera.LEFT;
    public int singleTagFiducialID = 1;
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record GlobalPoseObservation(
      double timestamp,
      Pose3d pose,
      Pose3d altPose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      double averageTagDistanceAlt,
      boolean useMultiTag) {}

  public static record SingleTagPoseObservation(
      double timestamp,
      Pose3d pose,
      Pose3d altPose,
      double ambiguity,
      double averageTagDistance,
      double averageTagDistanceAlt) {}

  public enum ObservationMode {
    GLOBAL_POSE,
    SINGLE_TAG_SINGLE_CAM
  }

  public enum SingleTagCamera {
    LEFT,
    RIGHT
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void enableGlobalPose() {}

  public default void enableSingleTagSingleCam(int tagID, SingleTagCamera side) {}
}
