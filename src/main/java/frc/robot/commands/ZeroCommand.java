package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class ZeroCommand extends Command {
  private final Superstructure superstructure;
  private final Drive drive;
  public boolean isSlow = false;

  public ZeroCommand(Superstructure superstructure, Drive drive) {
    this.superstructure = superstructure;
    this.drive = drive;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (Robot.alliance == Alliance.Blue) {
      drive.resetPose(new Pose2d());
    } else {
      drive.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
