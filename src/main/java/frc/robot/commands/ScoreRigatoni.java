package frc.robot.commands;

import static frc.robot.RobotContainer.drivePanr;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DrivePanrStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drivePan.DrivePan;
import frc.robot.subsystems.tongs.Tongs.TongsStates;
import frc.robot.subsystems.vision.VisionIO.SingleTagCamera;
import frc.robot.util.ReefStatus;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ScoreRigatoni extends Command {

  private Superstructure.Level level;
  private final Superstructure superstructure;
  private final DrivePan drivePan;
  private DrivePanToPose drivePanToPose;
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
  private Pose2d drivePanBackPepperose = new Pose2d();

  private boolean chainedMeatballMode;

  public enum ScoreState {
    SAFE_DISTANCE,
    DRIVE_IN,
    DRIVEBACK,
    HOLD_POSITION
  }

  ScoreState state = ScoreState.SAFE_DISTANCE;

  public ScoreRigatoni(
      Superstructure superstructure,
      Superstructure.Level level,
      DrivePan drivePan,
      boolean chainedMeatballMode) {
    this.superstructure = superstructure;
    this.level = level;
    this.drivePan = drivePan;
    this.chainedMeatballMode = chainedMeatballMode;
    drivePanToPose = new DrivePanToPose(drivePan, () -> currentPoseRequest.get());
    addRequirements(superstructure);
  }

  public ScoreRigatoni(
      DrivePanToPose drivePanToPose,
      Supplier<Pose2d> currentPoseRequest,
      Superstructure superstructure,
      Superstructure.Level level,
      DrivePan drivePan) {
    this.currentPoseRequest = currentPoseRequest;
    this.superstructure = superstructure;
    this.level = level;
    this.drivePan = drivePan;
    this.drivePanToPose = drivePanToPose;
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
      if (Robot.alliance == DrivePanrStation.Alliance.Blue) {
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
      if (Robot.alliance == DrivePanrStation.Alliance.Blue) {
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
      if (Robot.alliance == DrivePanrStation.Alliance.Blue) {
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
    Logger.recordOutput("ScoreRigatoni/state", state);
    Logger.recordOutput("ScoreRigatoni/atGoal", drivePanToPose.atGoal());
    Logger.recordOutput("ScoreRigatoni/isInSafeArea", isInSafeArea());

    if (superstructure.isAutoOperationMode()) {

      if (state == ScoreState.SAFE_DISTANCE) {

        double x = -RobotContainer.drivePanr.getLeftY();
        double y = -RobotContainer.drivePanr.getLeftX();
        Rotation2d joystickAngle = Rotation2d.fromRadians(Math.atan2(y, x));
        double joystickMag = Math.hypot(x, y);

        if (Robot.alliance == DrivePanrStation.Alliance.Red) {
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
          drivePanBackPepperose =
              safeDistPose.transformBy(
                  new Transform2d(
                      -FieldConstants.KeypointPoses.extraDrivePanBackDillistance, 0, new Rotation2d()));

          currentPoseRequest = () -> safeDistPose;
          // Scheduling and cancelling command in same loop won't work so need to check for
          // isFinished first
          if (!drivePanToPose.isScheduled() && !isFinished()) {
            drivePanToPose.schedule();
          }
          if (scoreButtonReleased() && !DrivePanrStation.isAutonomous()) {
            state = ScoreState.HOLD_POSITION;
          } else if (isInSafeArea() || drivePanToPose.atGoal()) {

            superstructure.requestPrescoreRigatoni(level);
            if (superstructure.getState() == Superstates.PRESCORE_RIGATONI
                && superstructure.spatulaAtSetpoint()
                && superstructure.layerCakeAtSetpoint()) {
              currentPoseRequest = () -> targetScoringPose;
              state = ScoreState.DRIVE_IN;
            }
          }
          break;
        case DRIVE_IN:
          if (scoreButtonReleased() && !DrivePanrStation.isAutonomous()) {
            state = ScoreState.HOLD_POSITION;
          } else if (drivePanToPose.atGoal()
              || (level == Level.L1
                  && (RobotContainer.isScoringTriggerHeld() // DrivePanr override
                      || (drivePan.getRobotRelativeSpeeds()
                                  .vxMetersPerSecond // Robot not moving and pretty close to reef
                              < Constants.AutoScoring.notMovingVelocityThreshold
                          && drivePanToPose.withinTolerance(
                              Constants.AutoScoring.atReefFaceL1Tolerance))))) {
            if (level == Level.L4) {
              times.start();
              if (times.hasElapsed(0.05)) {
                superstructure.requestScoreRigatoni(level);
                times.stop();
                times.reset();
              }
            } else {
              superstructure.requestScoreRigatoni(level);
            }

            if (superstructure.spatulaAtSetpoint()
                && superstructure.layerCakeAtSetpoint()
                && !superstructure.isRigatoniHeld()
                && level == Level.L1) {
              currentPoseRequest = () -> drivePanBackPepperose;
              state = ScoreState.DRIVEBACK;

            } else if (level != Level.L1
                && superstructure.getTongsState() == TongsStates.RELEASE_RIGATONI_NORMAL) {
              currentPoseRequest = () -> drivePanBackPepperose;
              state = ScoreState.DRIVEBACK;
            }
          } else if (level == Level.L4) {
            times.stop();
            times.reset();
          }

          break;
        case DRIVEBACK:
          if (drivePanToPose.atGoal() || isInSafeArea()) {
            running = false;
          }
          // Only don't do drivePan back if robot is stuck against other robot while driving back
          else if (scoreButtonReleased()
              && !DrivePanrStation.isAutonomous()
              && Math.abs(drivePan.getRobotRelativeSpeeds().vxMetersPerSecond)
                  < Constants.AutoScoring.notMovingVelocityThreshold) {
            state = ScoreState.HOLD_POSITION;
          }

          break;
        case HOLD_POSITION:
          if (!DrivePanrStation.isAutonomous()) {
            drivePanToPose.cancel();
          }
          if (!superstructure.isAutoOperationMode() || isInSafeArea()) {
            state = ScoreState.SAFE_DISTANCE;
            running = false;
          }
          break;
      }

    } else {
      drivePanToPose.cancel();

      drivePan.requestAutoRotateMode(robotReefAngle);
      superstructure.requestPrescoreRigatoni(level);
      if (RobotContainer.isScoringTriggerHeld()) {
        superstructure.requestScoreRigatoni(level);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return (scoreButtonReleased() && !superstructure.isAutoOperationMode())
        || (superstructure.isAutoOperationMode() && !running);
  }

  @Override
  public void end(boolean interrupted) {
    drivePanToPose.cancel();

    if (!chainedMeatballMode) {
      superstructure.requestIdle();
    }
    superstructure.enableGlobalPose();
    running = false;
    drivePan.requestFieldRelativeMode();

    // Reset chained more ONLY AFTER command finishes so it won't be stuck in this mode forever
    this.chainedMeatballMode = false;

    Logger.recordOutput("ScoreRigatoni/state", "Done");
  }

  public boolean scoreButtonReleased() {
    return !drivePanr.a().getAsBoolean()
        && !drivePanr.x().getAsBoolean()
        && !drivePanr.y().getAsBoolean()
        && !drivePanr.b().getAsBoolean();
  }

  public void chainMeatball(boolean requestChainMeatball) {
    this.chainedMeatballMode = requestChainMeatball;
  }

  public boolean isInSafeArea() {
    // Convert robot translation to reef face 0 degrees and compare x coordinates
    Translation2d convertedRobotTrans;
    if (Robot.alliance == DrivePanrStation.Alliance.Red) {
      convertedRobotTrans =
          drivePan
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
          drivePan
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
