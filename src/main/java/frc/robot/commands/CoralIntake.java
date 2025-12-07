package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.objectDetection.VisionObjectDetection;
import frc.robot.util.Trigon.simulatedfield.SimulatedGamePieceConstants.GamePieceType;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends DriveToPose {

  private IntakeSuperstructure intakeSuperstructure;
  private Drive drive;
  private VisionObjectDetection visionObjectDetection;
  private Translation2d coralPosition;
  private Pose2d driveToPoseTarget = Pose2d.kZero;
  private static Supplier<Pose2d> currentPoseRequest = () -> new Pose2d();
  private Rotation2d targetAngle;
  private Timer ejectAutoTimer = new Timer();

  public CoralIntake(
      IntakeSuperstructure intakeSuperstructure,
      Drive drive,
      VisionObjectDetection visionObjectDetection) {
    super(drive, false, false);
    super.initPose(() -> currentPoseRequest.get());
    this.intakeSuperstructure = intakeSuperstructure;
    this.drive = drive;
    this.visionObjectDetection = visionObjectDetection;

    addRequirements(intakeSuperstructure);
  }

  @Override
  public void initialize() {
    super.initialize();
    coralPosition = null;
    targetAngle = null;
    ejectAutoTimer.stop();
    ejectAutoTimer.reset();
  }

  @Override
  public void execute() {
    if (coralPosition == null) {
      coralPosition = visionObjectDetection.calculateBestObjectPositionOnField(GamePieceType.CORAL);
      if (coralPosition != null) {
        targetAngle =
            coralPosition
                .minus(drive.getPose().getTranslation())
                .getAngle()
                .plus(Rotation2d.k180deg);
        intakeSuperstructure.requestIntake();
      }
    }

    if (coralPosition != null) {
      Logger.recordOutput("CoralIntake/coralPosition", new Pose2d(coralPosition, Rotation2d.kZero));
      Logger.recordOutput(
          "CoralIntake/coralAngleBotRelativeDeg",
          coralPosition.minus(drive.getPose().getTranslation()).getAngle().getDegrees());
      Logger.recordOutput(
          "CoralIntake/driveHeadingDeg", drive.getPose().getRotation().getDegrees());
      Logger.recordOutput("CoralIntake/targetAngleDeg", targetAngle.getDegrees());

      switch (Constants.VisionObjectDetection.coralIntakeMode) {
        case MANUAL:
          break;
        case AUTO_ALIGN:
          drive.requestAutoRotateMode(targetAngle);
          break;
        case AUTO_ALIGN_DRIVE:
          driveToPoseTarget =
              new Pose2d(coralPosition, targetAngle)
                  .transformBy(
                      new Transform2d(
                          new Translation2d(Constants.VisionObjectDetection.coralIntakeOffset, 0),
                          new Rotation2d()));
          super.execute();
          currentPoseRequest = () -> driveToPoseTarget;

          if (super.atGoal()) {
            ejectAutoTimer
                .start(); // If it is at goal we want to start the timer for how long we continue
            // intaking
            if (ejectAutoTimer.hasElapsed(
                1.2)) { // After 0.2 seconds we intake coral again so that it can pick it back up
              intakeSuperstructure.requestIntake();
              ejectAutoTimer.stop();
              ejectAutoTimer.reset();
            } else if (ejectAutoTimer.hasElapsed(1)) { // After 1 second we eject coral
              intakeSuperstructure.requestEject();
            }
          }
          break;
        case AUTO_DRIVE:
          driveToPoseTarget = new Pose2d(coralPosition, drive.getRotation());
          currentPoseRequest = () -> driveToPoseTarget;
          super.execute();
          if (super.atGoal()) {
            ejectAutoTimer.start();
            if (ejectAutoTimer.hasElapsed(1.05)) {
              intakeSuperstructure.requestIntake();
              ejectAutoTimer.stop();
              ejectAutoTimer.reset();
            } else if (ejectAutoTimer.hasElapsed(1)) {
              intakeSuperstructure.requestEject();
            }
          }
          break;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return (intakeSuperstructure.isCoralDetectedIndexer() && DriverStation.isAutonomous())
        || intakeSuperstructure.isCoralDetectedPickupArea();
  }

  @Override
  public void end(boolean interrupted) {

    interrupted = true;
    drive.requestFieldRelativeMode();
  }
}
