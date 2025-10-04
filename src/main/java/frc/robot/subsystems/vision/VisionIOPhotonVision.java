package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.FieldConstants;
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final SingleTagCamera currentCam;

  private ObservationMode observationMode = ObservationMode.GLOBAL_POSE;
  private SingleTagCamera singleTagCamToUse = SingleTagCamera.LEFT;
  private int singleTagFiducialID = 1;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;

    if (name == VisionConstants.leftCamName) {
      currentCam = SingleTagCamera.LEFT;
    } else {
      currentCam = SingleTagCamera.RIGHT;
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    inputs.observationMode = observationMode;
    inputs.singleTagCamToUse = singleTagCamToUse;
    inputs.singleTagFiducialID = singleTagFiducialID;

    // Read new camera observations
    List<GlobalPoseObservation> globalPoseObservations = new LinkedList<>();
    List<SingleTagPoseObservation> singleTagPoseObservations = new LinkedList<>();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      switch (inputs.observationMode) {
        case GLOBAL_POSE:
          if (result.multitagResult.isPresent()) { // Multitag result
            MultiTargetPNPResult multitagResult = result.multitagResult.get();

            // Calculate robot pose
            Transform3d fieldToCamera = multitagResult.estimatedPose.best;
            Transform3d fieldToCameraAlt = multitagResult.estimatedPose.alt;

            Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
            Transform3d fieldToRobotAlt = fieldToCameraAlt.plus(robotToCamera.inverse());
            Pose3d robotPose =
                new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
            Pose3d altPose3d =
                new Pose3d(fieldToRobotAlt.getTranslation(), fieldToRobotAlt.getRotation());

            // Calculate average tag distance
            double totalTagDistance = 0.0;
            for (var target : result.targets) {
              totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
            }

            // Add observation
            globalPoseObservations.add(
                new GlobalPoseObservation(
                    result.getTimestampSeconds(), // Timestamp
                    robotPose,
                    altPose3d,
                    multitagResult.estimatedPose.ambiguity, // Ambiguity
                    multitagResult.fiducialIDsUsed.size(), // Tag count
                    totalTagDistance / result.targets.size(), // Average tag distance
                    true));

          } else if (!result.targets.isEmpty()) { // Single tag result
            var target = result.targets.get(0);

            // Calculate robot pose
            var tagPose = FieldConstants.aprilTagFieldLayout.getTagPose(target.fiducialId);
            if (tagPose.isPresent()) {
              Transform3d fieldToTarget =
                  new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());

              Transform3d cameraToTarget = target.bestCameraToTarget;
              Transform3d cameraToTargetAlt = target.altCameraToTarget;
              Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
              Transform3d fieldToCameraAlt = fieldToTarget.plus(cameraToTargetAlt.inverse());

              Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
              Transform3d fieldToRobotAlt = fieldToCameraAlt.plus(robotToCamera.inverse());

              Pose3d robotPose =
                  new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
              Pose3d robotPoseAlt =
                  new Pose3d(fieldToRobotAlt.getTranslation(), fieldToRobotAlt.getRotation());

              // Add observation
              globalPoseObservations.add(
                  new GlobalPoseObservation(
                      result.getTimestampSeconds(), // Timestamp
                      robotPose, // 3D pose estimate
                      robotPoseAlt,
                      target.poseAmbiguity, // Ambiguitys
                      1, // Tag count
                      cameraToTarget.getTranslation().getNorm(), // Average tag distance
                      false));
            }
          }
          break;
        case SINGLE_TAG_SINGLE_CAM:
          // Don't do processing unless using this camera in single tag mode
          if (currentCam != singleTagCamToUse) {
            break;
          }

          PhotonTrackedTarget target = null;
          if (!result.targets.isEmpty()) {
            for (PhotonTrackedTarget trackedTarget : result.getTargets()) {
              if (trackedTarget.fiducialId == inputs.singleTagFiducialID) {
                target = trackedTarget;
              }
            }

            // Skip if tag isn't detected
            if (target == null) {
              break;
            }

            var tagPose = FieldConstants.aprilTagFieldLayout.getTagPose(target.fiducialId);
            if (tagPose.isPresent()) {
              Transform3d fieldToTarget =
                  new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
              Transform3d cameraToTarget = target.bestCameraToTarget;
              Transform3d cameraToTargetAlt = target.altCameraToTarget;

              Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
              Transform3d fieldToCameraAlt = fieldToTarget.plus(cameraToTargetAlt.inverse());

              Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
              Transform3d fieldToRobotAlt = fieldToCameraAlt.plus(robotToCamera.inverse());

              Pose3d robotPose =
                  new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
              Pose3d robotPoseAlt =
                  new Pose3d(fieldToRobotAlt.getTranslation(), fieldToRobotAlt.getRotation());

              // Add observation
              singleTagPoseObservations.add(
                  new SingleTagPoseObservation(
                      result.getTimestampSeconds(), // Timestamp
                      robotPose, // 3D pose estimate
                      robotPoseAlt,
                      target.poseAmbiguity, // Ambiguity
                      cameraToTarget.getTranslation().getNorm())); // Average tag distance
            }
          }
          break;
      }
    }

    // Save pose observations to inputs object
    inputs.globalPoseObservations = new GlobalPoseObservation[globalPoseObservations.size()];
    for (int i = 0; i < globalPoseObservations.size(); i++) {
      inputs.globalPoseObservations[i] = globalPoseObservations.get(i);
    }

    inputs.singleTagPoseObservations =
        new SingleTagPoseObservation[singleTagPoseObservations.size()];
    for (int i = 0; i < singleTagPoseObservations.size(); i++) {
      inputs.singleTagPoseObservations[i] = singleTagPoseObservations.get(i);
    }
  }

  @Override
  public void enableGlobalPose() {
    observationMode = ObservationMode.GLOBAL_POSE;
  }

  @Override
  public void enableSingleTagSingleCam(int tagID, SingleTagCamera side) {
    observationMode = ObservationMode.SINGLE_TAG_SINGLE_CAM;
    singleTagFiducialID = tagID;
    singleTagCamToUse = side;
  }
}
