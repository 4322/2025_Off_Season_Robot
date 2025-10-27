/*
 * Code based off of FRC Team 5990 Trigon's object detection code
 * https://github.com/Programming-TRIGON/RobotCode2025/tree/main/src/main/java/frc/trigon/robot/misc/objectdetectioncamera
 */
// TODO auto drive to piece commmand; Drive while rotating; test if driving without rotating works
package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.Trigon.simulatedfield.SimulatedGamePieceConstants;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

// import org.trigon.hardware.RobotHardwareStats;

/**
 * An object detection camera is a class that represents a camera that detects objects other than
 * apriltags, most likely game pieces.
 */
public class VisionObjectDetection extends SubsystemBase {
  private final VisionObjectDetectionIO visionObjectDetectionIO;
  private final VisionObjectDetectionInputsAutoLogged visionObjectDetectionInputs =
      new VisionObjectDetectionInputsAutoLogged();
  private final Transform3d robotCenterToCamera;

  public VisionObjectDetection(VisionObjectDetectionIO io, Transform3d robotCenterToCamera) {
    this.visionObjectDetectionIO = io;
    this.robotCenterToCamera = robotCenterToCamera;
  }

  @Override
  public void periodic() {
    visionObjectDetectionIO.updateInputs(visionObjectDetectionInputs);
    Logger.recordOutput(
        "VisionObjectDetection/getClosestCoralPositionRelativeToCamera()",
        getClosestCoralPositionRelativeToCamera());
    Logger.recordOutput(
        "VisionObjectDetection/hasCoral",
        hasTargets(SimulatedGamePieceConstants.GamePieceType.CORAL));
    Logger.recordOutput(
        "VisionObjectDetection/hasAlgae",
        hasTargets(SimulatedGamePieceConstants.GamePieceType.ALGAE));
    Logger.processInputs("VisionObjectDetection", visionObjectDetectionInputs);
  }

  /**
   * Calculates the position of the best object on the field from the 3D rotation of the object
   * relative to the camera. This assumes the object is on the ground. Once it is known that the
   * object is on the ground, one can simply find the transform from the camera to the ground and
   * apply it to the object's rotation.
   *
   * @return the best object's 2D position on the field (z is assumed to be 0)
   */
  public Translation2d calculateBestObjectPositionRelativeToCamera(
      SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
    final Translation2d[] targetObjectsTranslation =
        getObjectPositionsRelativeToCamera(targetGamePiece);

    if (targetObjectsTranslation.length == 0) return null;

    Translation2d bestObjectTranslation = targetObjectsTranslation[0];

    for (int i = 1; i < targetObjectsTranslation.length; i++) {
      final Translation2d currentObjectTranslation = targetObjectsTranslation[i];
      final double bestObjectDistance = bestObjectTranslation.getNorm();
      final double currentObjectDistance = currentObjectTranslation.getNorm();

      if (currentObjectDistance < bestObjectDistance) {
        bestObjectTranslation = currentObjectTranslation;
      }
    }

    return bestObjectTranslation;
  }

  public boolean hasTargets(SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
    return visionObjectDetectionInputs.hasTarget[targetGamePiece.id];
  }

  public Translation2d[] getObjectPositionsOnField(
      SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
    final Rotation3d[] visibleObjectsRotations = getTargetObjectsRotations(targetGamePiece);
    final Translation2d[] objectsPositionsOnField =
        new Translation2d[visibleObjectsRotations.length];

    for (int i = 0; i < visibleObjectsRotations.length; i++)
      objectsPositionsOnField[i] = calculateObjectPositionFromRotation(visibleObjectsRotations[i]);

    Logger.recordOutput(
        "ObjectDetectionCamera/Visible" + targetGamePiece.name(), objectsPositionsOnField);
    return objectsPositionsOnField;
  }

  public Translation2d[] getObjectPositionsRelativeToCamera(
      SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
    final Rotation3d[] visibleObjectsRotations = getTargetObjectsRotations(targetGamePiece);
    if (visibleObjectsRotations == null || visibleObjectsRotations.length == 0) {
      return new Translation2d[0];
    }

    final Translation2d[] objectsPositionsRelativeToCamera =
        new Translation2d[visibleObjectsRotations.length];
    for (int i = 0; i < visibleObjectsRotations.length; i++) {
      objectsPositionsRelativeToCamera[i] =
          calculateObjectPositionFromRotation(visibleObjectsRotations[i]);
    }

    Logger.recordOutput(
        "ObjectDetectionCamera/Visible" + targetGamePiece.name(), objectsPositionsRelativeToCamera);
    return objectsPositionsRelativeToCamera;
  }

  public Rotation3d[] getTargetObjectsRotations(
      SimulatedGamePieceConstants.GamePieceType targetGamePiece) {
    if (targetGamePiece.id < 0
        || targetGamePiece.id >= visionObjectDetectionInputs.visibleObjectRotations.length) {
      return new Rotation3d[0];
    }

    if (targetGamePiece == SimulatedGamePieceConstants.GamePieceType.CORAL
        && Constants.VisionObjectDetection.SHOULD_IGNORE_LOLLIPOP_CORAL) {
      ArrayList<Rotation3d> rotations = new ArrayList<>();
      for (Rotation3d rotation :
          visionObjectDetectionInputs.visibleObjectRotations[targetGamePiece.id]) {
        if (!isLollipop(rotation.toRotation2d())) rotations.add(rotation);
      }
      return rotations.toArray(new Rotation3d[0]);
    }
    return visionObjectDetectionInputs.visibleObjectRotations[targetGamePiece.id];
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
  private Translation2d calculateObjectPositionFromRotation(
      Rotation3d objectRotation) { // TODO account for pitch in code or gui
    final double cameraHeight = robotCenterToCamera.getZ(); // Camera height above the ground
    final double objectPitchSin = Math.sin(objectRotation.getY());
    final double objectYaw = objectRotation.getZ();

    // TODO rotate by yaw
    if (Math.abs(objectPitchSin) < 0.000001) {
      return new Translation2d(); // Return a default or zero position
    }

    final double xTransform = cameraHeight / objectPitchSin;

    final Transform3d objectRotationToGround =
        new Transform3d(xTransform, 0, objectYaw, new Rotation3d());
    return objectRotationToGround.getTranslation().toTranslation2d();
  }

  private boolean isLollipop(Rotation2d objectYaw) {
    for (Rotation3d algaeYaw :
        getTargetObjectsRotations(SimulatedGamePieceConstants.GamePieceType.ALGAE)) {
      final double difference = Math.abs(algaeYaw.toRotation2d().minus(objectYaw).getDegrees());
      if (difference < Constants.VisionObjectDetection.LOLLIPOP_TOLERANCE.getDegrees()) return true;
    }
    return false;
  }

  public Translation2d getClosestCoralPositionRelativeToCamera() {
    // Get the closest coral's position relative to the camera
    return calculateBestObjectPositionRelativeToCamera(
        SimulatedGamePieceConstants.GamePieceType.CORAL);
  }
}
