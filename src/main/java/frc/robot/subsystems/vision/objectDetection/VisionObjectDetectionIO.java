package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLog;

public class VisionObjectDetectionIO {
  protected VisionObjectDetectionIO() {}

  protected void updateInputs(VisionObjectDetectionInputsAutoLogged inputs) {}

  @AutoLog
  public static class VisionObjectDetectionInputs {
    /**
     * Whether there is at least one target or not for each game piece, by game piece index (type).
     */
    public boolean[] hasTarget =
        new boolean[Constants.VisionObjectDetection.numberOfGamePieceTypes];
    /**
     * Stores the Rotation3d of all visible objects. The first index is the game piece ID (type).
     * The second index is the index of the game piece's Rotation3d, with the best object placed
     * first (index 0).
     */
    public Rotation3d[][] visibleObjectRotations =
        new Rotation3d[Constants.VisionObjectDetection.numberOfGamePieceTypes][0];

    public double latestResultTimestamp = 0;
  }
}
