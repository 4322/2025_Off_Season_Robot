/*
 * Code based off of FRC Team 5990 Trigon's object detection code
 * https://github.com/Programming-TRIGON/RobotCode2025/tree/main/src/main/java/frc/trigon/robot/misc/objectdetectioncamera
 */
// TODO auto drive to piece commmand; Drive while rotating; test if driving without rotating works
package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
// import frc.trigon.robot.commands.commandfactories.CoralCollectionCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Trigon.simulatedfield.SimulatedGamePieceConstants;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

/**
 * An object detection camera is a class that represents a camera that detects objects other than
 * apriltags, most likely game pieces.
 */
public class VisionObjectDetection extends SubsystemBase {
  private final VisionObjectDetectionInputsAutoLogged objectDetectionCameraInputs =
      new VisionObjectDetectionInputsAutoLogged();
  private final VisionObjectDetectionIO visionObjectDetectionIO;
  private final String hostname;
  private final Transform3d robotCenterToCamera;
  private Drive drive;

  public VisionObjectDetection(
      String hostname,
      Transform3d robotCenterToCamera,
      Drive drive,
      VisionObjectDetectionIO visionObjectDetectionIO) {
    this.hostname = hostname;
    this.robotCenterToCamera = robotCenterToCamera;
    this.visionObjectDetectionIO = visionObjectDetectionIO;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    visionObjectDetectionIO.updateInputs(objectDetectionCameraInputs);
    Logger.processInputs(hostname, objectDetectionCameraInputs);
    Logger.recordOutput(
        "ObjectDetectionCamera/HasTargetsCoral",
        hasTargets(SimulatedGamePieceConstants.GamePieceType.CORAL));
  }

  /**
   * Calculates the position of the best object on the field from the 3D rotation of the object
   * relative to the camera. This assumes the object is on the ground. Once it is known that the
   * object is on the ground, one can simply find the transform from the camera to the ground and
   * apply it to the object's rotation.
   *
   * @return the best object's 2D position on the field (z is assumed to be 0)
   */
  public Translation2d calculateBestObjectPositionOnField(
      SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
    final Translation2d[] targetObjectsTranslation = getObjectPositionsOnField(targetGamePiece);
    final Translation2d currentRobotTranslation = drive.getPose().getTranslation();
    if (targetObjectsTranslation.length == 0) return null;
    Translation2d bestObjectTranslation = targetObjectsTranslation[0];

    for (int i = 1; i < targetObjectsTranslation.length; i++) {
      final Translation2d currentObjectTranslation = targetObjectsTranslation[i];
      final double bestObjectDifference =
          currentRobotTranslation.getDistance(bestObjectTranslation);
      final double currentObjectDifference =
          currentRobotTranslation.getDistance(currentObjectTranslation);
      if (currentObjectDifference < bestObjectDifference)
        bestObjectTranslation = currentObjectTranslation;
    }
    return bestObjectTranslation;
  }

  public boolean hasTargets(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
    return objectDetectionCameraInputs.hasTarget[targetGamePiece.id];
  }

  public Translation2d[] getObjectPositionsOnField(
      SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
    final Rotation3d[] visibleObjectsRotations = getTargetObjectsRotations(targetGamePiece);
    final Translation2d[] objectsPositionsOnField =
        new Translation2d[visibleObjectsRotations.length];

    for (int i = 0; i < visibleObjectsRotations.length; i++) {
      objectsPositionsOnField[i] = calculateObjectPositionFromRotation(visibleObjectsRotations[i]);
      Logger.recordOutput(
          "ObjectDetectionCamera/Visible" + targetGamePiece.name(),
          new Pose2d(objectsPositionsOnField[i], new Rotation2d()));
    }

    return objectsPositionsOnField;
  }

  public Rotation3d[] getTargetObjectsRotations(
      SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
    if (targetGamePiece == SimulatedGamePieceConstants.GamePieceType.CORAL
        && Constants.VisionObjectDetection.shouldIgnoreLollipopCoral) {
      ArrayList<Rotation3d> rotations = new ArrayList<>();
      for (Rotation3d rotation :
          objectDetectionCameraInputs.visibleObjectRotations[targetGamePiece.id]) {
        if (!isLollipop(rotation.toRotation2d())) rotations.add(rotation);
      }
      return rotations.toArray(new Rotation3d[0]);
    }
    return objectDetectionCameraInputs.visibleObjectRotations[targetGamePiece.id];
  }

  /**
   * Calculates the position of the object on the field from the 3D rotation of the object relative
   * to the camera. This assumes the object is on the ground. Once it is known that the object is on
   * the ground, one can simply find the transform from the camera to the ground and apply it to the
   * object's rotation.
   *
   * @param objectRotation the object's 3D rotation relative to the camera
   * @return the object's 2D position on the field (z is assumed to be 0)
   */
  private Translation2d calculateObjectPositionFromRotation(Rotation3d objectRotation) {
    final Pose2d robotPoseAtResultTimestamp =
        drive.getPoseAtTimestamp(objectDetectionCameraInputs.latestResultTimestamp);
    if (robotPoseAtResultTimestamp == null) return new Translation2d();
    final Pose3d cameraPose = new Pose3d(robotPoseAtResultTimestamp).plus(robotCenterToCamera);
    final Pose3d objectRotationStart = cameraPose.plus(new Transform3d(0, 0, 0, objectRotation));

    final double cameraZ = cameraPose.getTranslation().getZ();
    final double objectPitchSin = Math.sin(objectRotationStart.getRotation().getY());
    final double xTransform = cameraZ / objectPitchSin;
    final Transform3d objectRotationStartToGround =
        new Transform3d(xTransform, 0, 0, new Rotation3d());

    return objectRotationStart
        .transformBy(objectRotationStartToGround)
        .getTranslation()
        .toTranslation2d();
  }

  private boolean isLollipop(Rotation2d objectYaw) {
    for (Rotation3d algaeYaw :
        getTargetObjectsRotations(SimulatedGamePieceConstants.GamePieceType.ALGAE)) {
      final double difference = Math.abs(algaeYaw.toRotation2d().minus(objectYaw).getDegrees());
      if (difference < Constants.VisionObjectDetection.lollipopTolerance.getDegrees()) return true;
    }
    return false;
  }

  public Translation2d calculateDistanceFromTrackedCoral() {
        Pose2d robotPose = drive.getPose();
        final Translation2d trackedObjectPositionOnField = calculateBestObjectPositionOnField(
            SimulatedGamePieceConstants.GamePieceType.CORAL);
        if (trackedObjectPositionOnField == null)
            return null;

        final Translation2d difference = robotPose.getTranslation().minus(trackedObjectPositionOnField);
        final Translation2d robotToTrackedCoralDistance = difference.rotateBy(robotPose.getRotation().unaryMinus());
        Logger.recordOutput("VisionObjectDetection/TrackedCoralDistance", robotToTrackedCoralDistance);
        return robotToTrackedCoralDistance;
    }



}
