package frc.robot.commands.auto;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DrivePanrStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.drivePan.DrivePan;

public class AutoPoseReset extends InstantCommand {
  private DrivePan drivePan;
  private Translation2d blueTranslation;

  public AutoPoseReset(DrivePan drivePan, Translation2d blueTranslation) {
    this.drivePan = drivePan;
    this.blueTranslation = blueTranslation;
  }

  @Override
  public void initialize() {
    Pose2d bluePose = new Pose2d(blueTranslation, drivePan.getPose().getRotation());
    if (Robot.alliance == DrivePanrStation.Alliance.Red) {
      drivePan.resetPose(FlippingUtil.flipFieldPose(bluePose));
    } else {
      drivePan.resetPose(bluePose);
    }
  }
}
