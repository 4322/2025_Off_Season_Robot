package frc.robot.subsystems.vision;

import java.security.spec.ECField;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.SingleTagCamera;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ReefStatus;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  public boolean ReefFaceAmbiguity;
  public boolean ReefPipeAmbiguity;
 private Drive drive;
  public Translation2d ReefCenterPoint;
  public Pose2d robotPose = drive.getPose();
  private  double convertedRobotTrans;
  private double reefFace;
  public enum ClosestReefPipe { // TODO
    LEFT,
    RIGHT
  }
  ClosestReefPipe closestReefPipe;

  public enum L1Zone {
    MIDDLE,
    LEFT,
    RIGHT
  }
  ClosestReefPipe reefPipe;
  public double reefToRobotDeg;


  private ObservationMode observationMode = ObservationMode.GLOBAL_POSE;
  private SingleTagCamera singleTagCamToUse = SingleTagCamera.LEFT;
  private int singleTagFiducialID = 1;

  private enum ObservationMode {
    GLOBAL_POSE,
    SINGLE_TAG_SINGLE_CAM,
    SINGLE_TAG_MULTI_CAM
  }

  private enum SingleTagCamera {
    LEFT,
    RIGHT
  }

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  public void enableGlobalPose() {
    for (VisionIO cameraIO : io) {
      cameraIO.enableGlobalPose();
    }
  }

  public void enableSingleTagSingleCam(int tagID, SingleTagCamera side) {
    for (VisionIO cameraIO : io) {
      cameraIO.enableSingleTagSingleCam(tagID, side);
    }
  }

  public void enableSingleTagMultiCam(int tagID) {
    for (VisionIO cameraIO : io) {
      cameraIO.enableSingleTagMultiCam(tagID);
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public void ReefStatus (){ 
    Translation2d robotTranslation = robotPose.getTranslation();
    Rotation2d reefCenterToRobotDeg = ReefCenterPoint.minus(robotTranslation).getAngle();
    reefToRobotDeg = reefCenterToRobotDeg.getRadians();
    if ( -30 <= reefToRobotDeg || reefToRobotDeg <= 30){ 
    reefFace = 0;
  } else if (30 < reefToRobotDeg && reefToRobotDeg <= 90
  ){ 
    reefFace = -60;
  } else if (-90  < reefToRobotDeg && reefToRobotDeg <= -30){ 
    reefFace = 60;
  } else if (90 < reefToRobotDeg || reefToRobotDeg <= 150){ 
    reefFace = -120;
  } else if (-150 < reefToRobotDeg && reefToRobotDeg <= -90){ 
    reefFace = 120;
  } else if (-210 < reefToRobotDeg && reefToRobotDeg < -210){ 
    reefFace = 180;
  }

  drive.requestAutoRotateMode(reefFace);

  //convertedRobotTrans = robotTranslation.rotateAround(ReefCenterPoint, );

  if ( -30 <= reefToRobotDeg || reefToRobotDeg <= 0){ 
  closestReefPipe = ClosestReefPipe.LEFT;
} else if ( 0 <= reefToRobotDeg || reefToRobotDeg <= 30){ 
  closestReefPipe = ClosestReefPipe.RIGHT;
} 

if (-30 < reefToRobotDeg || reefToRobotDeg < -10)  {

  L1Zone l1Zone = L1Zone.LEFT; // TODO

}
else if (-10 < reefToRobotDeg || reefToRobotDeg < 10)  {

 L1Zone l1Zone = L1Zone.MIDDLE; // TODO

}
else if (10 < reefToRobotDeg || reefToRobotDeg < 30){
  L1Zone l1Zone = L1Zone.RIGHT;
}
}
}
