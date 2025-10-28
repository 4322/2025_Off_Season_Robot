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

public class CoralIntake extends Command {

  private IntakeSuperstructure intakeSuperstructure;
  private Drive drive;
  private VisionObjectDetection visionObjectDetection;
  private Superstructure superstructure;
  private Translation2d coralPosition;
  private Transform2d coralTransform;
  private DriveToPose driveToPose;
  private Timer rumbleTimer = new Timer();

  public CoralIntake(
      IntakeSuperstructure intakeSuperstructure,
      Drive drive,
      VisionObjectDetection visionObjectDetection,
      Superstructure superstructure) {
    this.intakeSuperstructure = intakeSuperstructure;
    this.drive = drive;
    this.visionObjectDetection = visionObjectDetection;

    rumbleTimer.reset();
    rumbleTimer.stop();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    coralPosition = visionObjectDetection.getClosestCoralPositionRelativeToCamera();
    intakeSuperstructure.requestIntake();
    coralPosition = visionObjectDetection.getClosestCoralPositionRelativeToCamera();
    if (coralPosition != null) {
      coralTransform = new Transform2d(coralPosition, Rotation2d.kZero);
      driveToPose =
          new DriveToPose(drive, superstructure.getRobotPoseEstimate().plus(coralTransform));
      if (!rumbleTimer.isRunning()) {
        rumbleTimer.reset();
        rumbleTimer.start();
        driver.setRumble(RumbleType.kBothRumble, 1);
      } else {

      }
      // TODO Math/methods for turning towards coral
    }
  }

  @Override
  public boolean isFinished() {
    return (driveToPose != null && driveToPose.atGoal())
        || !(driver.getLeftTriggerAxis() > 0.5)
        || intakeSuperstructure.isCoralDetectedIndexer();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSuperstructure.requestRetractIdle();
    driver.setRumble(RumbleType.kBothRumble, 0);
  }
}
