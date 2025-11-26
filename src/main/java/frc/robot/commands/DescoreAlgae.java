package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ReefStatus;
import frc.robot.util.ReefStatus.AlgaeLevel;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DescoreAlgae extends DriveToPose {
  private final Superstructure superstructure;
  private final Drive drive;
  public boolean running;
  private AlgaeLevel reefLevel;

  private Pose2d targetScoringPose;
  private Rotation2d robotReefAngle;
  private ReefStatus reefStatus;
  private Supplier<Pose2d> currentPoseRequest = () -> new Pose2d();
  private Pose2d safeDescorePose;
  private Pose2d driveBackPose;

  public enum ScoreState {
    SAFE_DISTANCE,
    DRIVE_IN,
    DRIVEBACK,
    HOLD_POSITION
  }

  private Superstructure.Level level;
  private ScoreState state = ScoreState.SAFE_DISTANCE;

  public DescoreAlgae(Superstructure superstructure, Drive drive) {
    super(drive, false);
    this.superstructure = superstructure;
    this.drive = drive;
    setPoseSupplier(() -> currentPoseRequest.get());
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {

    running = true;
    state = ScoreState.SAFE_DISTANCE;

    reefStatus = superstructure.getReefStatus();
    robotReefAngle = reefStatus.getClosestRobotAngle();
    reefLevel = reefStatus.getAlgaeLevel();

    if (reefLevel == ReefStatus.AlgaeLevel.L2) {
      level = Superstructure.Level.L2;
    } else {
      level = Superstructure.Level.L3;
    }

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

    safeDescorePose =
        targetScoringPose.transformBy(
            new Transform2d(
                -FieldConstants.KeypointPoses.safeDistFromAlgaeDescorePos, 0, new Rotation2d()));

    driveBackPose =
        safeDescorePose.transformBy(
            new Transform2d(
                -FieldConstants.KeypointPoses.extraDriveBackDistance, 0, new Rotation2d()));
    super.initialize();
  }

  @Override
  public void execute() {
    Logger.recordOutput("DescoreAlgae/State", state);
    Logger.recordOutput("DescoreAlgae/atGoal", atGoal());
    Logger.recordOutput("DescoreAlgae/isInSafeArea", isInSafeArea());
    if (superstructure.isAutoOperationMode()) {
      switch (state) {
        case SAFE_DISTANCE:
          this.currentPoseRequest = () -> safeDescorePose;
          // Scheduling and cancelling command in same loop won't work so need to check for
          // isFinished first
          super.execute();

          if (descoreButtonReleased() && !DriverStation.isAutonomous()) {
            state = ScoreState.HOLD_POSITION;
          } else if (isInSafeArea() || atGoal()) {
            superstructure.requestDescoreAlgae(level);

            if (superstructure.getState() == Superstates.DESCORE_ALGAE
                && superstructure.armAtSetpoint()
                && superstructure.elevatorAtSetpoint()
                && withinTolerance(Constants.AutoScoring.algaeSafeDistTolerance)) {
              state = ScoreState.DRIVE_IN;
              this.currentPoseRequest = () -> targetScoringPose;
              resetGoal();
            }
          }
          break;
        case DRIVE_IN:
          super.execute();
          if (descoreButtonReleased() && !DriverStation.isAutonomous()) {
            state = ScoreState.HOLD_POSITION;
          } else if (superstructure.isAlgaeHeld()) {
            this.currentPoseRequest = () -> driveBackPose;
            resetGoal();
            state = ScoreState.DRIVEBACK;
          }
          break;
        case DRIVEBACK:
          super.execute();
          if (descoreButtonReleased() && !DriverStation.isAutonomous()) {
            state = ScoreState.HOLD_POSITION;
          }
          if (atGoal() || isInSafeArea()) {
            running = false;
          }

          break;
        case HOLD_POSITION:
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
    running = false;

    super.end(interrupted);
    Logger.recordOutput("DescoreAlgae/State", "Done");
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
