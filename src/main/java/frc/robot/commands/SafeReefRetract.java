package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ReefStatus;

public class SafeReefRetract extends Command {
  private final Superstructure superstructure;
  private Drive drive;
  private ScoreCoral scoreCoral;
  private RobotContainer RobotContainer;
  private Rotation2d robotReefAngle;
  private ReefStatus reefStatus;

  public SafeReefRetract(Superstructure superstructure, Drive drive) {
    this.superstructure = superstructure;
    this.drive = drive;
  }

  @Override
  public void initialize() {
    reefStatus = superstructure.getReefStatus();

    robotReefAngle = reefStatus.getClosestRobotAngle();
  }

  @Override
  public boolean isFinished() {
    return isInSafeArea();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
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
          >= FieldConstants.KeypointPoses.reefSafeDistance;
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
          >= FieldConstants.KeypointPoses.reefSafeDistance;
    }
  }
}
