package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class AlgaeScoreCommand extends Command {
  private Superstructure superstructure;
  private Drive drive;

  public AlgaeScoreCommand(Superstructure superstructure, Drive drive) {
    this.superstructure = superstructure;
    this.drive = drive;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.requestAlgaePrescore();
    if (superstructure.scoreBackSideBarge()) {
      if (Robot.alliance == DriverStation.Alliance.Red) {
        drive.requestAutoRotateMode(Rotation2d.fromDegrees(0));
      } else {
        drive.requestAutoRotateMode(Rotation2d.fromDegrees(180));
      }
    } else {
      if (Robot.alliance == DriverStation.Alliance.Blue) {
        drive.requestAutoRotateMode(Rotation2d.fromDegrees(0));
      } else {
        drive.requestAutoRotateMode(Rotation2d.fromDegrees(180));
      }
    }
  }

  @Override
  public void execute() {

    if (RobotContainer.isScoringTriggerHeld() && superstructure.isAlgaeHeld()) {
      superstructure.requestAlgaeScore();
    }
  }

  @Override
  public boolean isFinished() {
    return !driver.b().getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
    drive.requestFieldRelativeMode();
  }
}
