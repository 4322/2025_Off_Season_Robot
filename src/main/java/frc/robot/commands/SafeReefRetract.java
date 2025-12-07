package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;

public class SafeReefRetract extends Command {
    private Drive drive;
    private Rotation2d robotReefAngle;
    private Superstructure superstructure;
    
    public SafeReefRetract(Drive drive, Superstructure superstructure) {
        this.drive = drive;
        this.superstructure = superstructure;
    }
    @Override
    public void initialize() {
        robotReefAngle = superstructure.getReefStatus().getClosestRobotAngle();
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