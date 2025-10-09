package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ReefStatus;
import java.util.function.Supplier;

public class DescoreAlgae extends Command {
  private final Superstructure superstructure;
  private final Drive drive;
  private DriveToPose driveToPose;
  public boolean running;

  private Pose2d targetScoringPose;
  private Rotation2d robotReefAngle;
  private ReefStatus reefStatus;
  private Supplier<Pose2d> currentPoseRequest = () -> new Pose2d();
  private Pose2d safeDescorePose;

  public enum ScoreState {
    SAFE_DISTANCE,
    DRIVE_IN,
    DRIVEBACK,
    HOLD_POSITION
  }

  private Superstructure.Level level;
  private ScoreState state = ScoreState.SAFE_DISTANCE;

  public DescoreAlgae(Superstructure superstructure, Superstructure.Level level, Drive drive) {
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

    reefStatus = superstructure.getReefStatus();
    robotReefAngle = reefStatus.getClosestRobotAngle();

    if (Robot.alliance == DriverStation.Alliance.Blue) {
      targetScoringPose =
          new Pose2d(
              FieldConstants.KeypointPoses.descoreAlgaeDriveInBlue.rotateAround(
                  FieldConstants.KeypointPoses.blueReefCenter,
                  robotReefAngle.rotateBy(Rotation2d.k180deg)),
              robotReefAngle);
    } else {
      targetScoringPose =
          new Pose2d(
              FieldConstants.KeypointPoses.descoreAlgaeDriveInRed.rotateAround(
                  FieldConstants.KeypointPoses.redReefCenter,
                  robotReefAngle.rotateBy(Rotation2d.k180deg)),
              robotReefAngle);
    }

    // TODO: Make sure safe dist is the same for Algae as it is for coral
    safeDescorePose =
        targetScoringPose.transformBy(
            new Transform2d(
                FieldConstants.KeypointPoses.safeDistFromAlgaeDescorePos,
                0,
                robotReefAngle.rotateBy(Rotation2d.k180deg)));
  }

  @Override
  public void execute() {

    if (superstructure.isAutoOperationMode()) {
      switch (state) {
        case SAFE_DISTANCE:
          currentPoseRequest = () -> safeDescorePose;
          if (!driveToPose.isScheduled()) {
            driveToPose.schedule();
          }
          if (isInSafeArea() || driveToPose.atGoal()) {
            superstructure.requestDescoreAlgae(level);
          }

          if (superstructure.armAtSetpoint()
              && superstructure.elevatorAtSetpoint()
              && driveToPose.atGoal()) {
            state = ScoreState.DRIVE_IN;
          }
          break;
        case DRIVE_IN:
          currentPoseRequest = () -> targetScoringPose;

          if (descoreButtonReleased()) {
            state = ScoreState.HOLD_POSITION;
          }
          if (superstructure.armAtSetpoint() && superstructure.elevatorAtSetpoint()) {
            state = ScoreState.DRIVEBACK;
          }
          break;
        case DRIVEBACK:
          currentPoseRequest = () -> safeDescorePose;
          if (descoreButtonReleased()) {
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
      superstructure.requestDescoreAlgae(level);
    }
  }

  public boolean descoreButtonReleased() {
    return !driver.y().getAsBoolean() && !driver.x().getAsBoolean();
  }

  @Override
  public boolean isFinished() {
    return ((descoreButtonReleased() && !superstructure.isAutoOperationMode())
        || (superstructure.isAutoOperationMode() && !running));
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
    drive.requestFieldRelativeMode();
    running = false;
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
