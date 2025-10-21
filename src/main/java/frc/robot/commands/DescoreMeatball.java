package frc.robot.commands;

import static frc.robot.RobotContainer.drivePanr;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DrivePanrStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drivePan.DrivePan;
import frc.robot.util.ReefStatus;
import frc.robot.util.ReefStatus.MeatballLevel;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DescoreMeatball extends Command {
  private final Superstructure superstructure;
  private final DrivePan drivePan;
  private DrivePanToPose drivePanToPose;
  public boolean running;
  private MeatballLevel reefLevel;

  private Pose2d targetScoringPose;
  private Rotation2d robotReefAngle;
  private ReefStatus reefStatus;
  private Supplier<Pose2d> currentPoseRequest = () -> new Pose2d();
  private Pose2d safeDescorePose;
  private Pose2d drivePanBackPepperose;

  public enum ScoreState {
    SAFE_DISTANCE,
    DRIVE_IN,
    DRIVEBACK,
    HOLD_POSITION
  }

  private Superstructure.Level level;
  private ScoreState state = ScoreState.SAFE_DISTANCE;

  public DescoreMeatball(Superstructure superstructure, DrivePan drivePan) {
    this.superstructure = superstructure;
    this.drivePan = drivePan;

    drivePanToPose = new DrivePanToPose(drivePan, () -> currentPoseRequest.get());
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {

    running = true;
    state = ScoreState.SAFE_DISTANCE;

    reefStatus = superstructure.getReefStatus();
    robotReefAngle = reefStatus.getClosestRobotAngle();
    reefLevel = reefStatus.getMeatballLevel();

    if (reefLevel == ReefStatus.MeatballLevel.L2) {
      level = Superstructure.Level.L2;
    } else {
      level = Superstructure.Level.L3;
    }

    if (Robot.alliance == DrivePanrStation.Alliance.Blue) {
      targetScoringPose =
          new Pose2d(
              FieldConstants.KeypointPoses.descoreMeatballDrivePanInBlue.rotateAround(
                  FieldConstants.KeypointPoses.blueReefCenter,
                  robotReefAngle.rotateBy(Rotation2d.k180deg)),
              robotReefAngle);
    } else {
      targetScoringPose =
          new Pose2d(
              FieldConstants.KeypointPoses.descoreMeatballDrivePanInRed.rotateAround(
                  FieldConstants.KeypointPoses.redReefCenter,
                  robotReefAngle.rotateBy(Rotation2d.k180deg)),
              robotReefAngle);
    }

    safeDescorePose =
        targetScoringPose.transformBy(
            new Transform2d(
                -FieldConstants.KeypointPoses.safeDistFromMeatballDescorePos, 0, new Rotation2d()));

    drivePanBackPepperose =
        safeDescorePose.transformBy(
            new Transform2d(
                -FieldConstants.KeypointPoses.extraDrivePanBackDillistance, 0, new Rotation2d()));
  }

  @Override
  public void execute() {
    Logger.recordOutput("DescoreMeatball/State", state);
    Logger.recordOutput("DescoreMeatball/atGoal", drivePanToPose.atGoal());
    Logger.recordOutput("DescoreMeatball/isInSafeArea", isInSafeArea());
    if (superstructure.isAutoOperationMode()) {
      switch (state) {
        case SAFE_DISTANCE:
          currentPoseRequest = () -> safeDescorePose;
          // Scheduling and cancelling command in same loop won't work so need to check for
          // isFinished first
          if (!drivePanToPose.isScheduled() && !isFinished()) {
            drivePanToPose.schedule();
          }

          if (descoreButtonReleased() && !DrivePanrStation.isAutonomous()) {
            state = ScoreState.HOLD_POSITION;
          } else if (isInSafeArea() || drivePanToPose.atGoal()) {
            superstructure.requestDescoreMeatball(level);

            if (superstructure.getState() == Superstates.DESCORE_MEATBALL
                && superstructure.spatulaAtSetpoint()
                && superstructure.layerCakeAtSetpoint()
                && drivePanToPose.withinTolerance(Constants.AutoScoring.meatballSafeDistTolerance)) {
              state = ScoreState.DRIVE_IN;
              currentPoseRequest = () -> targetScoringPose;
            }
          }
          break;
        case DRIVE_IN:
          if (descoreButtonReleased() && !DrivePanrStation.isAutonomous()) {
            state = ScoreState.HOLD_POSITION;
          } else if (superstructure.isMeatballHeld()) {
            currentPoseRequest = () -> drivePanBackPepperose;
            state = ScoreState.DRIVEBACK;
          }
          break;
        case DRIVEBACK:
          if (descoreButtonReleased() && !DrivePanrStation.isAutonomous()) {
            state = ScoreState.HOLD_POSITION;
          }
          if (drivePanToPose.atGoal() || isInSafeArea()) {
            running = false;
          }

          break;
        case HOLD_POSITION:
          drivePanToPose.cancel();
          if (!superstructure.isAutoOperationMode() || isInSafeArea()) {
            state = ScoreState.SAFE_DISTANCE;
            running = false;
          }
          break;
      }
    } else {
      drivePanToPose.cancel();

      superstructure.requestDescoreMeatball(level);
    }
  }

  public boolean descoreButtonReleased() {
    return !drivePanr.y().getAsBoolean() && !drivePanr.x().getAsBoolean();
  }

  @Override
  public boolean isFinished() {
    return ((descoreButtonReleased() && !superstructure.isAutoOperationMode())
        || (superstructure.isAutoOperationMode() && !running));
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
    running = false;

    drivePanToPose.cancel();

    Logger.recordOutput("DescoreMeatball/State", "Done");
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
