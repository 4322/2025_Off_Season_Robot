package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.SingleTagCamera;
import frc.robot.util.ReefStatus;
import java.util.function.Supplier;

public class ScoreCoral extends Command {

  private Superstructure.Level level;
  private final Superstructure superstructure;
  private final Drive drive;
  private DriveToPose driveToPose;
  public boolean running;

  private Pose2d targetScoringPose;
  private Supplier<Pose2d> currentPoseRequest = () -> new Pose2d();

  private Pose2d leftBranchScoringPos;
  private Pose2d rightBranchScoringPose;

  private Pose2d leftTroughScoringPose;
  private Pose2d middleTroughScoringPose;
  private Pose2d rightTroughScoringPose;

  private Rotation2d robotReefAngle;
  private ReefStatus reefStatus;

  public enum ScoreState {
    SAFE_DISTANCE,
    DRIVE_IN,
    DRIVEBACK,
    HOLD_POSITION
  }

  private enum DriveToPoseTesting {
    SAFE_DISTANCE,
    DRIVE_IN,
    DRIVEBACK,
  }

  ScoreState state = ScoreState.SAFE_DISTANCE;

  DriveToPoseTesting driveToPoseState = DriveToPoseTesting.SAFE_DISTANCE;

  public ScoreCoral(Superstructure superstructure, Superstructure.Level level, Drive drive) {
    this.superstructure = superstructure;
    this.level = level;
    this.drive = drive;

    driveToPose = new DriveToPose(drive, () -> currentPoseRequest.get());
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    running = true;
    state = ScoreState.SAFE_DISTANCE;

    // Fill in poses of all possible scoring locations for the level requested
    reefStatus = superstructure.getReefStatus();
    robotReefAngle = reefStatus.getClosestRobotAngle();

    if (level == Level.L1) {
      if (Robot.alliance == DriverStation.Alliance.Blue) {
        leftTroughScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.leftTroughScoringBlue.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        middleTroughScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.middleTroughScoringBlue.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        rightTroughScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.rightTroughScoringBlue.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
      } else {
        leftTroughScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.leftTroughScoringRed.rotateAround(
                    FieldConstants.KeypointPoses.redReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        middleTroughScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.middleTroughScoringRed.rotateAround(
                    FieldConstants.KeypointPoses.redReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        rightTroughScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.rightTroughScoringRed.rotateAround(
                    FieldConstants.KeypointPoses.redReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
      }

      switch (reefStatus.getClosestL1Zone()) {
        case LEFT:
          targetScoringPose = leftTroughScoringPose;
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
          break;
        case MIDDLE:
          targetScoringPose = middleTroughScoringPose;
          superstructure.enableGlobalPose();
          break;
        case RIGHT:
          targetScoringPose = rightTroughScoringPose;
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
          break;
      }

    } else {
      if (Robot.alliance == DriverStation.Alliance.Blue) {
        leftBranchScoringPos =
            new Pose2d(
                FieldConstants.KeypointPoses.leftReefBranchScoringBlue.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        rightBranchScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.rightReefBranchScoringBlue.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
      } else {
        leftBranchScoringPos =
            new Pose2d(
                FieldConstants.KeypointPoses.leftReefBranchScoringRed.rotateAround(
                    FieldConstants.KeypointPoses.redReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        rightBranchScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.rightReefBranchScoringRed.rotateAround(
                    FieldConstants.KeypointPoses.redReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
      }

      switch (reefStatus.getClosestReefPipe()) {
        case LEFT:
          targetScoringPose = leftBranchScoringPos;
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
          break;
        case RIGHT:
          targetScoringPose = rightBranchScoringPose;
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
          break;
      }
    }
  }

  @Override
  public void execute() {

    if (Constants.enableDriveToPoseTestingScoreCoral) {
      Pose2d safeDistPose =
          targetScoringPose.transformBy(
              new Transform2d(
                  FieldConstants.KeypointPoses.safeDistFromCoralScoringPos,
                  0,
                  robotReefAngle.rotateBy(Rotation2d.k180deg)));
      switch (driveToPoseState) {
        case SAFE_DISTANCE:
          if (!driveToPose.isScheduled()) {
            driveToPose.schedule();
          }
          currentPoseRequest = () -> safeDistPose;
          if (Constants.enableDriveToPoseWithPrescore) {
            superstructure.requestPrescoreCoral(level);
          }
          if ((isInSafeArea() || driveToPose.atGoal()) && RobotContainer.isScoringTriggerHeld()) {
            driveToPoseState = DriveToPoseTesting.DRIVE_IN;
          }
          break;
        case DRIVE_IN:
          currentPoseRequest = () -> leftBranchScoringPos;
          if (driver.povRight().getAsBoolean() && driveToPose.atGoal()) {
            driveToPoseState = DriveToPoseTesting.SAFE_DISTANCE;
          }
          break;
      }

      if (driver.povLeft().getAsBoolean()) {
        if (driveToPose.isScheduled()) {
          driveToPose.cancel();
        }
        running = false;
      }

      if (driveToPoseState == DriveToPoseTesting.SAFE_DISTANCE) {

        double x = -RobotContainer.driver.getLeftY();
        double y = -RobotContainer.driver.getLeftX();
        Rotation2d joystickAngle = Rotation2d.fromRadians(Math.atan2(y, x));
        double joystickMag = Math.hypot(x, y);

        if (level == Level.L1) {

          if (joystickMag > 0.75) {
            if (joystickAngle.getDegrees() > 30) {
              targetScoringPose = leftTroughScoringPose;
              superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
            } else if (joystickAngle.minus(robotReefAngle).getDegrees() < -30) {
              targetScoringPose = rightTroughScoringPose;
              superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
            } else {
              targetScoringPose = middleTroughScoringPose;
              superstructure.enableGlobalPose();
            }
          }

        } else {

          if (joystickMag > 0.75) {
            if (joystickAngle.getDegrees() > 0) {
              targetScoringPose = leftBranchScoringPos;
              superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
            } else if (joystickAngle.getDegrees() < 0) {
              targetScoringPose = rightBranchScoringPose;
              superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
            }
          }
        }
      }
    } else if (superstructure.isAutoOperationMode()
        && !Constants.enableDriveToPoseTestingScoreCoral) {
      if (state == ScoreState.SAFE_DISTANCE) {

        double x = -RobotContainer.driver.getLeftY();
        double y = -RobotContainer.driver.getLeftX();
        Rotation2d joystickAngle = Rotation2d.fromRadians(Math.atan2(y, x));
        double joystickMag = Math.hypot(x, y);

        if (Robot.alliance == DriverStation.Alliance.Red) {
          joystickAngle =
              joystickAngle.rotateBy(Rotation2d.k180deg); // Convert joystick to field relative
        }

        // Rotate joystick to be relative to robot 0 degrees
        joystickAngle = joystickAngle.rotateBy(robotReefAngle.unaryMinus());

        if (level == Level.L1) {

          if (joystickMag > 0.75) {
            if (joystickAngle.getDegrees() > 30) {
              targetScoringPose = leftTroughScoringPose;
              superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
            } else if (joystickAngle.minus(robotReefAngle).getDegrees() < -30) {
              targetScoringPose = rightTroughScoringPose;
              superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
            } else {
              targetScoringPose = middleTroughScoringPose;
              superstructure.enableGlobalPose();
            }
          }

        } else {

          if (joystickMag > 0.75) {
            if (joystickAngle.getDegrees() > 0) {
              targetScoringPose = leftBranchScoringPos;
              superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
            } else if (joystickAngle.getDegrees() < 0) {
              targetScoringPose = rightBranchScoringPose;
              superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
            }
          }
        }
      }
      Pose2d safeDistPose =
          targetScoringPose.transformBy(
              new Transform2d(
                  FieldConstants.KeypointPoses.safeDistFromCoralScoringPos,
                  0,
                  robotReefAngle.rotateBy(Rotation2d.k180deg)));

      switch (state) {
        case SAFE_DISTANCE:
          currentPoseRequest = () -> safeDistPose;
          if (!driveToPose.isScheduled()) {
            driveToPose.schedule();
          }
          if (isInSafeArea() || driveToPose.atGoal()) {
            superstructure.requestPrescoreCoral(level);
          }

          if (superstructure.armAtSetpoint() && superstructure.elevatorAtSetpoint()) {
            state = ScoreState.DRIVE_IN;
          }
          break;
        case DRIVE_IN:
          currentPoseRequest = () -> targetScoringPose;

          if (scoreButtonReleased()) {
            state = ScoreState.HOLD_POSITION;
          } else if (driveToPose.atGoal()) {
            superstructure.requestScoreCoral(level);
            if (superstructure.armAtSetpoint() && superstructure.elevatorAtSetpoint()) {
              state = ScoreState.DRIVEBACK;
            }
          }
          break;
        case DRIVEBACK:
          currentPoseRequest = () -> safeDistPose;
          if (scoreButtonReleased()) {
            state = ScoreState.HOLD_POSITION;
          }
          if (driveToPose.atGoal()) {
            running = false;
          }

          break;
        case HOLD_POSITION:
          if (driveToPose.isScheduled()) {
            driveToPose.cancel();
          }
          if (!superstructure.isAutoOperationMode() || isInSafeArea()) {
            state = ScoreState.SAFE_DISTANCE;
            running = false;
          }
          break;
      }

    } else {
      if (driveToPose.isScheduled()) {
        driveToPose.cancel();
      }

      superstructure.requestPrescoreCoral(level);

      if (RobotContainer.isScoringTriggerHeld()) {
        superstructure.requestScoreCoral(level);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return (scoreButtonReleased()
            && !superstructure.isAutoOperationMode()
            && !Constants.enableDriveToPoseTestingScoreCoral)
        || (superstructure.isAutoOperationMode()
            && !running
            && !Constants.enableDriveToPoseTestingScoreCoral)
        || (Constants.enableDriveToPoseTestingScoreCoral && !running);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
    superstructure.enableGlobalPose();
    running = false;

    if (driveToPose.isScheduled()) {
      driveToPose.cancel();
    }
  }

  public boolean scoreButtonReleased() {
    return !driver.a().getAsBoolean()
        && !driver.x().getAsBoolean()
        && !driver.y().getAsBoolean()
        && !driver.b().getAsBoolean();
  }

  public boolean isInSafeArea() {
    Translation2d convertedRobotTrans;
    if (Robot.alliance == DriverStation.Alliance.Red) {
      convertedRobotTrans =
          drive
              .getPose()
              .getTranslation()
              .rotateAround(
                  FieldConstants.KeypointPoses.redReefCenter,
                  robotReefAngle.rotateBy(Rotation2d.k180deg).unaryMinus());
      return (convertedRobotTrans.getX()
              - FieldConstants.KeypointPoses.leftReefBranchFaceRed.getX())
          > FieldConstants.KeypointPoses.reefSafeDistance;
    } else {
      convertedRobotTrans =
          drive
              .getPose()
              .getTranslation()
              .rotateAround(
                  FieldConstants.KeypointPoses.blueReefCenter,
                  robotReefAngle.rotateBy(Rotation2d.k180deg).unaryMinus());
      return (convertedRobotTrans.getX()
              - FieldConstants.KeypointPoses.leftReefBranchFaceBlue.getX())
          > FieldConstants.KeypointPoses.reefSafeDistance;
    }
  }
}
