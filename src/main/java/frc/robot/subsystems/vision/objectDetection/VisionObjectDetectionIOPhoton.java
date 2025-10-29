package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionObjectDetectionIOPhoton extends VisionObjectDetectionIO {
  private final PhotonCamera photonCamera;

  public VisionObjectDetectionIOPhoton(String hostname) {
    PhotonCamera.setVersionCheckEnabled(false);
    photonCamera = new PhotonCamera(hostname);
  }

  @Override
  protected void updateInputs(VisionObjectDetectionInputsAutoLogged inputs) {
    if (!photonCamera.isConnected()) {
      updateNoNewResultInputs(inputs);
      return;
    }

    final PhotonPipelineResult result = getLatestPipelineResult();
    if (result == null || !result.hasTargets()) {
      updateNoNewResultInputs(inputs);
      return;
    }

    updateHasNewResultInputs(inputs, result);
  }

  private PhotonPipelineResult getLatestPipelineResult() {
    final List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
    return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
  }

  private void updateNoNewResultInputs(VisionObjectDetectionInputsAutoLogged inputs) {
    inputs.hasTarget = new boolean[Constants.VisionObjectDetection.numberOfGamePieceTypes];
    inputs.visibleObjectRotations =
        new Rotation3d[Constants.VisionObjectDetection.numberOfGamePieceTypes][0];
  }

  private void updateHasNewResultInputs(
      VisionObjectDetectionInputsAutoLogged inputs, PhotonPipelineResult result) {
    final List<Rotation3d>[] visibleObjectsRotations =
        new List[Constants.VisionObjectDetection.numberOfGamePieceTypes];
    for (int i = 0; i < Constants.VisionObjectDetection.numberOfGamePieceTypes; i++)
      visibleObjectsRotations[i] = new ArrayList<>();
    Arrays.fill(inputs.hasTarget, false);
    inputs.latestResultTimestamp = result.getTimestampSeconds();

    for (PhotonTrackedTarget currentTarget : result.getTargets()) {
      if (currentTarget.getDetectedObjectClassID() == -1) continue;

      inputs.hasTarget[currentTarget.getDetectedObjectClassID()] = true;
      visibleObjectsRotations[currentTarget.getDetectedObjectClassID()].add(
          extractRotation3d(currentTarget));
    }

    for (int i = 0; i < Constants.VisionObjectDetection.numberOfGamePieceTypes; i++)
      inputs.visibleObjectRotations[i] = toArray(visibleObjectsRotations[i]);
  }

  private Rotation3d[] toArray(List<Rotation3d> list) {
    final Rotation3d[] array = new Rotation3d[list.size()];

    for (int i = 0; i < array.length; i++) array[i] = list.get(i);

    return array;
  }

  private Rotation3d extractRotation3d(PhotonTrackedTarget target) {
    return new Rotation3d(
        0, Units.degreesToRadians(-target.getPitch()), Units.degreesToRadians(-target.getYaw()));
  }
}
