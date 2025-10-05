package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.maxAmbiguity;
import static frc.robot.subsystems.vision.VisionConstants.maxZError;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.GlobalPoseObservation;
import frc.robot.subsystems.vision.VisionIO.SingleTagCamera;
import frc.robot.subsystems.vision.VisionIO.SingleTagPoseObservation;
import frc.robot.util.GeomUtil;
import frc.robot.util.PolynomialRegression;
import frc.robot.util.ReefStatus;
import frc.robot.util.ReefStatus.ClosestReefPipe;
import frc.robot.util.ReefStatus.L1Zone;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Drive drive;
  private static Translation2d redReefCenterPoint;
  private static Translation2d blueReefCenterPoint;
  private static Translation2d redLeftL1Split;
  private static Translation2d redRightL1Split;
  private static Translation2d blueLeftL1Split;
  private static Translation2d blueRightL1Split;
  public boolean reefFaceAmbiguity;
  public boolean reefPipeAmbiguity;
  private double reefFace;
  public ClosestReefPipe closestReefPipe;

  private PolynomialRegression xyStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.14, 0.17, 0.27, 0.38},
          2);
  private PolynomialRegression thetaStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068},
          1);

  public Vision(Drive drive, VisionIO... io) {
    this.drive = drive;
    this.consumer = drive::addVisionMeasurement;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
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
    List<Pose2d> allRobotPoses = new LinkedList<>();
    List<Pose2d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose2d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Initialize logging values
      List<Pose2d> robotPoses = new LinkedList<>();
      List<Pose2d> robotPosesAccepted = new LinkedList<>();
      List<Pose2d> robotPosesRejected = new LinkedList<>();

      switch (inputs[cameraIndex].observationMode) {
        case GLOBAL_POSE:
          for (GlobalPoseObservation observation : inputs[cameraIndex].globalPoseObservations) {
            Pose3d disambiguatedRobotPose; // Robot pose chosen after disambiguation

            // Disambiguate to select a single robot pose
            if (observation.useMultiTag()) {
              disambiguatedRobotPose = observation.pose();
            } else {
              if (observation.ambiguity() < VisionConstants.maxAmbiguity) {
                disambiguatedRobotPose = observation.pose();
              } else if (Math.abs(
                      observation
                          .pose()
                          .toPose2d()
                          .getRotation()
                          .minus(drive.getRotation())
                          .getRadians())
                  < Math.abs(
                      observation
                          .altPose()
                          .toPose2d()
                          .getRotation()
                          .minus(drive.getRotation())
                          .getRadians())) {
                disambiguatedRobotPose = observation.pose();
              } else {
                disambiguatedRobotPose = observation.altPose();
              }
            }

            // Check whether to reject pose
            boolean rejectPose =
                observation.tagCount() == 0 // Must have at least one tag
                    || (observation.tagCount() == 1
                        && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                    || Math.abs(disambiguatedRobotPose.getZ())
                        > maxZError // Must have realistic Z coordinate

                    // Must be within the field boundaries
                    || disambiguatedRobotPose.getX() < -0.5
                    || disambiguatedRobotPose.getX() > FieldConstants.fieldLength + 0.5
                    || disambiguatedRobotPose.getY() < -0.5
                    || disambiguatedRobotPose.getY() > FieldConstants.fieldWidth + 0.5;

            // Add pose to log
            robotPoses.add(disambiguatedRobotPose.toPose2d());
            if (rejectPose) {
              robotPosesRejected.add(disambiguatedRobotPose.toPose2d());
            } else {
              robotPosesAccepted.add(disambiguatedRobotPose.toPose2d());
            }

            // Skip if rejected
            if (rejectPose) {
              continue;
            }

            // Calculate standard deviations
            double xyStdDev = 0.0;
            double thetaStdDev = 0.0;

            if (observation.useMultiTag()) {
              xyStdDev = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
              thetaStdDev =
                  Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();

              consumer.accept(
                  disambiguatedRobotPose.toPose2d(),
                  observation.timestamp(),
                  VecBuilder.fill(
                      VisionConstants.stdDevBaseline
                          * VisionConstants.thetaStdDevBaseline
                          * xyStdDev,
                      VisionConstants.stdDevBaseline
                          * VisionConstants.thetaStdDevBaseline
                          * xyStdDev,
                      VisionConstants.stdDevBaseline
                          * VisionConstants.thetaStdDevBaseline
                          * thetaStdDev));
            } else {
              xyStdDev =
                  Math.max(xyStdDevModel.predict(observation.averageTagDistance()), 0.000001);
              thetaStdDev =
                  Math.max(thetaStdDevModel.predict(observation.averageTagDistance()), 0.000001);

              consumer.accept(
                  disambiguatedRobotPose.toPose2d(),
                  observation.timestamp(),
                  VecBuilder.fill(
                      VisionConstants.stdDevBaseline * xyStdDev,
                      VisionConstants.stdDevBaseline * xyStdDev,
                      VisionConstants.stdDevBaseline * thetaStdDev));
            }
          }
          break;
        case SINGLE_TAG_SINGLE_CAM:
          for (SingleTagPoseObservation observation :
              inputs[cameraIndex].singleTagPoseObservations) {
            Pose3d disambiguatedRobotPose; // Robot pose chosen after disambiguation

            // Disambiguate to select a single robot pose
            if (observation.ambiguity() < VisionConstants.maxAmbiguity) {
              disambiguatedRobotPose = observation.pose();
            } else if (Math.abs(
                    observation
                        .pose()
                        .toPose2d()
                        .getRotation()
                        .minus(drive.getRotation())
                        .getRadians())
                < Math.abs(
                    observation
                        .altPose()
                        .toPose2d()
                        .getRotation()
                        .minus(drive.getRotation())
                        .getRadians())) {
              disambiguatedRobotPose = observation.pose();
            } else {
              disambiguatedRobotPose = observation.altPose();
            }

            Pose2d visionRobotPose = disambiguatedRobotPose.toPose2d();
            Pose2d tagPos =
                FieldConstants.aprilTagFieldLayout
                    .getTagPose(inputs[cameraIndex].singleTagFiducialID)
                    .get()
                    .toPose2d();
            // Use gyro to correct for vision errors
            Rotation2d robotThetaError = drive.getRotation().minus(visionRobotPose.getRotation());
            Pose2d tagToRobotPose = drive.getPose().relativeTo(tagPos);

            visionRobotPose =
                tagPos.transformBy(
                    GeomUtil.poseToTransform(tagToRobotPose.rotateBy(robotThetaError)));

            // Check whether to reject pose
            boolean rejectPose =
                observation.ambiguity() > maxAmbiguity // Cannot be high ambiguity
                    || Math.abs(disambiguatedRobotPose.getZ())
                        > maxZError // Use Z coordinate of disambiguated robot pose due to vision
                    // pose math being done in 2d

                    // Must be within the field boundaries
                    || visionRobotPose.getX() < -0.5
                    || visionRobotPose.getX()
                        > FieldConstants.aprilTagFieldLayout.getFieldLength() + 0.5
                    || visionRobotPose.getY() < -0.5
                    || visionRobotPose.getY()
                        > FieldConstants.aprilTagFieldLayout.getFieldWidth() + 0.5;

            // Add pose to log
            robotPoses.add(visionRobotPose);
            if (rejectPose) {
              robotPosesRejected.add(visionRobotPose);
            } else {
              robotPosesAccepted.add(visionRobotPose);
            }

            // Skip if rejected
            if (rejectPose) {
              continue;
            }

            // Calculate standard deviations
            double xyStdDev =
                Math.max(xyStdDevModel.predict(observation.averageTagDistance()), 0.000001);

            consumer.accept(
                visionRobotPose,
                observation.timestamp(),
                VecBuilder.fill(
                    VisionConstants.stdDevBaseline * xyStdDev,
                    VisionConstants.stdDevBaseline * xyStdDev,
                    4322.0));
          }
          break;
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose2d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose2d[robotPosesRejected.size()]));
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose2d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose2d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose2d[allRobotPosesRejected.size()]));
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

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public ReefStatus getReefStatus() {
    Translation2d reefCenterPoint;
    Translation2d leftL1Split;
    Translation2d rightL1Split;
    if (Robot.alliance == DriverStation.Alliance.Red) {
      reefCenterPoint = redReefCenterPoint;
      leftL1Split = redLeftL1Split;
      rightL1Split = redRightL1Split;
    } else {
      reefCenterPoint = blueReefCenterPoint;
      leftL1Split = blueLeftL1Split;
      rightL1Split = blueRightL1Split;
    }
    Translation2d robotTranslation = drive.getPose().getTranslation();
    double reefCenterToRobotDeg = reefCenterPoint.minus(robotTranslation).getAngle().getDegrees();
    Translation2d convertedRobotTrans;

    if (-30 < reefCenterToRobotDeg && reefCenterToRobotDeg <= 30) {
      reefFace = -180;
      reefFaceAmbiguity = false;
    } else if (30 < reefCenterToRobotDeg && reefCenterToRobotDeg <= 90) {
      reefFace = -120;
      reefFaceAmbiguity = false;
    } else if (90 < reefCenterToRobotDeg && reefCenterToRobotDeg <= 150) {
      reefFace = -60;
      reefFaceAmbiguity = false;
    } else if ((150 < reefCenterToRobotDeg && reefCenterToRobotDeg <= 180)
        || (-150 < reefCenterToRobotDeg && reefCenterToRobotDeg <= -180)) {
      reefFace = 0;
      reefFaceAmbiguity = false;
    } else if (-150 < reefCenterToRobotDeg && reefCenterToRobotDeg <= -90) {
      reefFace = 60;
      reefFaceAmbiguity = false;
    } else if (-90 < reefCenterToRobotDeg && reefCenterToRobotDeg <= -30) {
      reefFace = 120;
      reefFaceAmbiguity = false;
    } else {
      reefFaceAmbiguity = true;
    }
    L1Zone l1Zone;
    // // Provide a valid Rotation2d argument, for example Rotation2d.fromRadians(reefToRobotDeg)
    convertedRobotTrans =
        robotTranslation.rotateAround(
            reefCenterPoint, Rotation2d.fromDegrees(-reefCenterToRobotDeg));

    if (reefCenterPoint.minus(convertedRobotTrans).getAngle().getDegrees()
        >= 0) { // Make sure it is for each face
      closestReefPipe = ClosestReefPipe.LEFT;
    } else {
      closestReefPipe = ClosestReefPipe.RIGHT;
    }

    if (leftL1Split.minus(convertedRobotTrans).getAngle().getDegrees()
        >= 0) { // TODO: MAke it so its for indiviual faces
      l1Zone = L1Zone.LEFT; // TODO
    } else if (rightL1Split.minus(convertedRobotTrans).getAngle().getDegrees() <= 0) {
      l1Zone = L1Zone.RIGHT;
    } else {
      l1Zone = L1Zone.MIDDLE;
    }
    return new ReefStatus(
        reefFaceAmbiguity,
        reefPipeAmbiguity,
        Rotation2d.fromDegrees(reefFace),
        closestReefPipe,
        l1Zone);
  }
}
