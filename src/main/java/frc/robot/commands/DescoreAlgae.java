package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ReefStatus;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DescoreAlgae extends Command {
  private final Superstructure superstructure;
  private final Drive drive;
  private DriveToPose driveToPose;
  public boolean running;
  public Timer times = new Timer();

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
    times.stop();
    times.reset();
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
                -FieldConstants.KeypointPoses.safeDistFromAlgaeDescorePos, 0, new Rotation2d()));
  }

  @Override
  public void execute() {
    Logger.recordOutput("DescoreAlgae/State", state);
    Logger.recordOutput("DscoreAlgae/atGoal", driveToPose.atGoal());
    Logger.recordOutput("DscoreAlgae/isInSafeArea", isInSafeArea());
    if (superstructure.isAutoOperationMode()) {
      switch (state) {
        case SAFE_DISTANCE:
          currentPoseRequest = () -> safeDescorePose;
          if (!driveToPose.isScheduled()) {
            driveToPose.schedule();
          }

          if (descoreButtonReleased() && !DriverStation.isAutonomous()) {
            state = ScoreState.HOLD_POSITION;
          } else if (isInSafeArea() || driveToPose.atGoal()) {
            times.start();
            superstructure.requestDescoreAlgae(level);

            if (superstructure.getState() == Superstates.DESCORE_ALGAE
                && superstructure.armAtSetpoint()
                && superstructure.elevatorAtSetpoint()
                && driveToPose.atGoal()) {
              state = ScoreState.DRIVE_IN;
              currentPoseRequest = () -> targetScoringPose;
            }
          }
          break;
        case DRIVE_IN:
          if (descoreButtonReleased() && !DriverStation.isAutonomous()) {
            state = ScoreState.HOLD_POSITION;
          } else if (superstructure.isAlgaeHeld()) {
            currentPoseRequest = () -> safeDescorePose;
            state = ScoreState.DRIVEBACK;
          }
          break;
        case DRIVEBACK:
          if (descoreButtonReleased() && !DriverStation.isAutonomous()) {
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
    running = false;
    if (driveToPose.isScheduled()) {
      driveToPose.cancel();
    }
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
