package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.VisionObjectDetection.CoralIntakeMode;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.objectDetection.VisionObjectDetection;
import frc.robot.util.Trigon.simulatedfield.SimulatedGamePieceConstants.GamePieceType;
import java.util.function.Supplier;

public class CoralIntake extends Command {

  private IntakeSuperstructure intakeSuperstructure;
  private Drive drive;
  private VisionObjectDetection visionObjectDetection;
  private Translation2d coralPosition;
  private Pose2d driveToPoseTarget = Pose2d.kZero;
  private DriveToPose driveToPose;
  private Supplier<Pose2d> currentPoseRequest = () -> new Pose2d();
  private Rotation2d targetAngle;

  public CoralIntake(
      IntakeSuperstructure intakeSuperstructure,
      Drive drive,
      VisionObjectDetection visionObjectDetection) {
    this.intakeSuperstructure = intakeSuperstructure;
    this.drive = drive;
    this.visionObjectDetection = visionObjectDetection;
    driveToPose = new DriveToPose(drive, () -> currentPoseRequest.get(), false);

    addRequirements(intakeSuperstructure);
  }

  @Override
  public void initialize() {
    coralPosition = visionObjectDetection.calculateBestObjectPositionOnField(GamePieceType.CORAL);
    intakeSuperstructure.requestIntake();
  }

  @Override
  public void execute() {
    if (Constants.VisionObjectDetection.coralIntakeMode != CoralIntakeMode.MANUAL
        && coralPosition != null
        && !driveToPose.isScheduled()) {
      if (Constants.VisionObjectDetection.coralIntakeMode == CoralIntakeMode.AUTO_ALIGN_DRIVE
          && coralPosition != null) {
        targetAngle =
            coralPosition
                .minus(drive.getPose().getTranslation())
                .getAngle()
                .plus(Rotation2d.k180deg);
        driveToPoseTarget = new Pose2d(coralPosition, targetAngle);
      } else {
        driveToPoseTarget = new Pose2d(coralPosition, drive.getRotation());
      }

      currentPoseRequest = () -> driveToPoseTarget;

      if (Constants.VisionObjectDetection.coralIntakeMode != CoralIntakeMode.AUTO_ALIGN) {
        driveToPose.schedule();
      } else {
        drive.requestAutoRotateMode(targetAngle);
      }

    } else if (coralPosition == null) {
      coralPosition = visionObjectDetection.calculateBestObjectPositionOnField(GamePieceType.CORAL);
    }
  }

  @Override
  public boolean isFinished() {
    return intakeSuperstructure.isCoralDetectedIndexer();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSuperstructure.requestRetractIdle();
    driveToPose.cancel();
    drive.requestFieldRelativeMode();
  }
}
