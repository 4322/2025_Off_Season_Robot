package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.objectDetection.VisionObjectDetection;
import frc.robot.util.Trigon.simulatedfield.SimulatedGamePieceConstants.GamePieceType;

public class CoralIntake extends Command {

  private IntakeSuperstructure intakeSuperstructure;
  private Drive drive;
  private VisionObjectDetection visionObjectDetection;
  private Superstructure superstructure;
  private Translation2d coralPosition;
  private Transform2d coralTransform;
  private DriveToPose driveToPose;

  public CoralIntake(
      IntakeSuperstructure intakeSuperstructure,
      Drive drive,
      VisionObjectDetection visionObjectDetection,
      Superstructure superstructure) {
    this.intakeSuperstructure = intakeSuperstructure;
    this.drive = drive;
    this.visionObjectDetection = visionObjectDetection;

  }

  @Override
  public void initialize() {
    coralPosition = visionObjectDetection.calculateBestObjectPositionOnField(GamePieceType.CORAL);
    intakeSuperstructure.requestIntake();
  }

  @Override
  public void execute() {
   
    if (coralPosition != null && driveToPose == null) {
      coralTransform = new Transform2d(coralPosition, Rotation2d.kZero);
      driveToPose =
          new DriveToPose(drive, () -> superstructure.getRobotPoseEstimate().plus(coralTransform));
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
