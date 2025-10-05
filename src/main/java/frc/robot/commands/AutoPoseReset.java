package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;

public class AutoPoseReset extends InstantCommand {
  private Drive drive;
  private Translation2d blueTranslation;

  public AutoPoseReset(Drive drive, Translation2d blueTranslation) {
    this.drive = drive;
    this.blueTranslation = blueTranslation;
  }

  @Override
  public void initialize() {
    Pose2d bluePose = new Pose2d(blueTranslation, drive.getPose().getRotation());
    if (Robot.alliance == DriverStation.Alliance.Red) {
      drive.resetPose(FlippingUtil.flipFieldPose(bluePose));
    } else {
      drive.resetPose(bluePose);
    }
  }
}
