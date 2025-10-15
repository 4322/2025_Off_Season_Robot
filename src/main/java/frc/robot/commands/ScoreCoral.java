package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import static frc.robot.RobotContainer.driver;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endEffector.EndEffector.EndEffectorStates;
import frc.robot.subsystems.vision.VisionIO.SingleTagCamera;
import frc.robot.util.ReefStatus;

public class ScoreCoral extends Command {

  private Superstructure.Level level;
  private final Superstructure superstructure;
  private final Drive drive;
  private DriveToPose driveToPose;
  public boolean running;
  public Timer times = new Timer();
  private Pose2d targetScoringPose;
  private Supplier<Pose2d> currentPoseRequest = () -> new Pose2d();

  private Pose2d leftBranchScoringPos;
  private Pose2d rightBranchScoringPose;

  private Pose2d leftTroughScoringPose;
  private Pose2d middleTroughScoringPose;
  private Pose2d rightTroughScoringPose;

  private Rotation2d robotReefAngle;
  private ReefStatus reefStatus;
  private Pose2d safeDistPose = new Pose2d();

  private boolean chainedAlgaeMode;

  public enum ScoreState {
    SAFE_DISTANCE,
    DRIVE_IN,
    DRIVEBACK,
    HOLD_POSITION
  }

  ScoreState state = ScoreState.SAFE_DISTANCE;

  public ScoreCoral(
      Superstructure superstructure,
      Superstructure.Level level,
      Drive drive,
      boolean chainedAlgaeMode) {
    this.superstructure = superstructure;
    this.level = level;
    this.drive = drive;
    this.chainedAlgaeMode = chainedAlgaeMode;
    driveToPose = new DriveToPose(drive, () -> currentPoseRequest.get());
    addRequirements(superstructure);
  }

  public ScoreCoral(
      DriveToPose driveToPose,
      Supplier<Pose2d> currentPoseRequest,
      Superstructure superstructure,
      Superstructure.Level level,
      Drive drive) {
    this.currentPoseRequest = currentPoseRequest;
    this.superstructure = superstructure;
    this.level = level;
    this.drive = drive;
    this.driveToPose = driveToPose;
  }

  @Override
  public void initialize() {
    running = true;
    times.stop();
    times.reset();
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
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
          break;
        case MIDDLE:
          targetScoringPose = middleTroughScoringPose;
          superstructure.enableGlobalPose();
          break;
        case RIGHT:
          targetScoringPose = rightTroughScoringPose;
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
          break;
      }

    } else if (level == Level.L4) {
      if (Robot.alliance == DriverStation.Alliance.Blue) {
        leftBranchScoringPos =
            new Pose2d(
                FieldConstants.KeypointPoses.leftReefBranchScoringBlueL4.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        rightBranchScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.rightReefBranchScoringBlueL4.rotateAround(
                    FieldConstants.KeypointPoses.blueReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
      } else {
        leftBranchScoringPos =
            new Pose2d(
                FieldConstants.KeypointPoses.leftReefBranchScoringRedL4.rotateAround(
                    FieldConstants.KeypointPoses.redReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
        rightBranchScoringPose =
            new Pose2d(
                FieldConstants.KeypointPoses.rightReefBranchScoringRedL4.rotateAround(
                    FieldConstants.KeypointPoses.redReefCenter,
                    robotReefAngle.rotateBy(Rotation2d.k180deg)),
                robotReefAngle);
      }
      switch (reefStatus.getClosestReefPipe()) {
        case LEFT:
          targetScoringPose = leftBranchScoringPos;
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
          break;
        case RIGHT:
          targetScoringPose = rightBranchScoringPose;
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
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
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
          break;
        case RIGHT:
          targetScoringPose = rightBranchScoringPose;
          superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
          break;
      }
    }
  }

  @Override
  public void execute() {
    Logger.recordOutput("ScoreCoral/state", state);
    Logger.recordOutput("ScoreCoral/atGoal", driveToPose.atGoal());
    Logger.recordOutput("ScoreCoral/isInSafeArea", isInSafeArea());

    if (RobotContainer.isScoringTriggerHeld()) {
      superstructure.requestScoreCoral(level);
    }

    if (superstructure.isAutoOperationMode()) {

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
              superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
            } else if (joystickAngle.minus(robotReefAngle).getDegrees() < -30) {
              targetScoringPose = rightTroughScoringPose;
              superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
            } else {
              targetScoringPose = middleTroughScoringPose;
              superstructure.enableGlobalPose();
            }
          }

        } else {

          if (joystickMag > 0.75) {
            if (joystickAngle.getDegrees() > 0) {
              targetScoringPose = leftBranchScoringPos;
              superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.RIGHT);
            } else if (joystickAngle.getDegrees() < 0) {
              targetScoringPose = rightBranchScoringPose;
              superstructure.enableSingleTag(reefStatus.getFaceTagId(), SingleTagCamera.LEFT);
            }
          }
        }
      }

      switch (state) {
        case SAFE_DISTANCE:
          safeDistPose =
              targetScoringPose.transformBy(
                  new Transform2d(
                      level == Level.L1
                          ? -FieldConstants.KeypointPoses.safeDistFromTroughScoringPos
                          : -FieldConstants.KeypointPoses.safeDistFromBranchScoringPos,
                      0,
                      new Rotation2d()));

          currentPoseRequest = () -> safeDistPose;
          if (!driveToPose.isScheduled()) {
            driveToPose.schedule();
          }
          if (scoreButtonReleased() && !DriverStation.isAutonomous()) {
            state = ScoreState.HOLD_POSITION;
          } else if (isInSafeArea() || driveToPose.atGoal()) {

            superstructure.requestPrescoreCoral(level);
            if (superstructure.getState() == Superstates.PRESCORE_CORAL
                && superstructure.armAtSetpoint()
                && superstructure.elevatorAtSetpoint()) {
              currentPoseRequest = () -> targetScoringPose;
              state = ScoreState.DRIVE_IN;
            }
          }
          break;
        case DRIVE_IN:
          if (scoreButtonReleased() && !DriverStation.isAutonomous()) {
            state = ScoreState.HOLD_POSITION;
          } else if (driveToPose.atGoal()) {
            if (level == Level.L4) {
              times.start();
              if (times.hasElapsed(0.1)) {
                superstructure.requestScoreCoral(level);
                times.stop();
                times.reset();
              }
            } else {
              superstructure.requestScoreCoral(level);
            }
            if (superstructure.armAtSetpoint()
                && superstructure.elevatorAtSetpoint()
                && !superstructure.isCoralHeld()
                && level == Level.L1) {
              currentPoseRequest = () -> safeDistPose;
              state = ScoreState.DRIVEBACK;

            } else if (level != Level.L1
                && superstructure.getEndEffectorState() == EndEffectorStates.RELEASE_CORAL_NORMAL) {
              currentPoseRequest = () -> safeDistPose;
              state = ScoreState.DRIVEBACK;
            }
          } else if (level == Level.L4) {
            times.stop();
            times.reset();
          }

          break;
        case DRIVEBACK:
          if (scoreButtonReleased() && !DriverStation.isAutonomous()) {
            state = ScoreState.HOLD_POSITION;
          }
          if (driveToPose.atGoal()) {
            running = false;
          }

          break;
        case HOLD_POSITION:
          if (driveToPose.isScheduled() && !DriverStation.isAutonomous()) {
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
    }
  }

  @Override
  public boolean isFinished() {
    return (scoreButtonReleased() && !superstructure.isAutoOperationMode())
        || (superstructure.isAutoOperationMode() && !running);
  }

  @Override
  public void end(boolean interrupted) {
    if (!chainedAlgaeMode) {
      superstructure.requestIdle();
    }
    superstructure.enableGlobalPose();
    running = false;

    if (driveToPose.isScheduled()) {
      driveToPose.cancel();
    }
    Logger.recordOutput("ScoreCoral/state", "Done");
  }

  public boolean scoreButtonReleased() {
    return !driver.a().getAsBoolean()
        && !driver.x().getAsBoolean()
        && !driver.y().getAsBoolean()
        && !driver.b().getAsBoolean();
  }

  public boolean isInSafeArea() {
    // Convert robot translation to reef face 0 degrees and compare x coordinates
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
