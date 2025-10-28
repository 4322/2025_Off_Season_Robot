package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.objectDetection.VisionObjectDetection;
import frc.robot.util.Trigon.simulatedfield.SimulatedGamePieceConstants.GamePieceType;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends Command {

  private IntakeSuperstructure intakeSuperstructure;
  private Drive drive;
  private VisionObjectDetection visionObjectDetection;
  private Superstructure superstructure;
  private Translation2d coralPosition;
  private Pose2d driveToPoseTarget;
  private Pose2d coralPose2d;
  private DriveToPose driveToPose;
  private Supplier<Pose2d> currentPoseRequest = () -> new Pose2d();

  public CoralIntake(
      IntakeSuperstructure intakeSuperstructure,
      Drive drive,
      VisionObjectDetection visionObjectDetection,
      Superstructure superstructure) {
    this.intakeSuperstructure = intakeSuperstructure;
    this.drive = drive;
    this.visionObjectDetection = visionObjectDetection;
    driveToPose = new DriveToPose(drive, () -> currentPoseRequest.get());
  }

  @Override
  public void initialize() {
    coralPosition = visionObjectDetection.calculateBestObjectPositionOnField(GamePieceType.CORAL);
    intakeSuperstructure.requestIntake();
  }

  @Override
  public void execute() {
    driveToPoseTarget = currentPoseRequest.get();
    Logger.recordOutput("CoralIntakeCommand/coralPostionExists", coralPosition != null);
    Logger.recordOutput("CoralIntakeCommand/driveToPoseExists", driveToPose != null);
    Logger.recordOutput("CoralIntakeCommand/atGoal", driveToPose != null && driveToPose.atGoal());
    if (coralPosition != null) {
      Logger.recordOutput("CoralIntakeCommand/coralPosition", coralPosition);
    }
    if (driveToPoseTarget != null) {
      Logger.recordOutput("CoralIntakeCommand/driveToPoseTarget", driveToPoseTarget);
    }

    if (coralPosition != null && !driveToPose.isScheduled()) {
      coralPose2d = new Pose2d(coralPosition, new Rotation2d());
      currentPoseRequest = () -> driveToPoseTarget;
      driveToPose.schedule();
      // TODO Math/methods for turning towards coral
    } else {
      coralPosition = visionObjectDetection.calculateBestObjectPositionOnField(GamePieceType.CORAL);
    }
  }

  @Override
  public boolean isFinished() {
    return (driveToPose != null && driveToPose.atGoal())
        || intakeSuperstructure.isCoralDetectedIndexer();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSuperstructure.requestRetractIdle();
    driveToPose.cancel();
  }
}
