package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.objectDetection.VisionObjectDetection;

public class CoralIntake extends Command {

  private IntakeSuperstructure intakeSuperstructure;
  private Drive drive;
  private VisionObjectDetection visionObjectDetection;
  private Translation2d coralPosition;

  public CoralIntake(
      IntakeSuperstructure intakeSuperstructure,
      Drive drive,
      VisionObjectDetection visionObjectDetection) {
    this.intakeSuperstructure = intakeSuperstructure;
    this.drive = drive;
    this.visionObjectDetection = visionObjectDetection;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    coralPosition = visionObjectDetection.getClosestCoralPositionRelativeToCamera();
    intakeSuperstructure.requestIntake();
    if (coralPosition != null) {
      // TODO vibrate controller
      // TODO Math/methods for turning/driving towards coral
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intakeSuperstructure.requestRetractIdle();
  }
}
