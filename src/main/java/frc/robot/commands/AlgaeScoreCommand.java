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
    if (Math.abs(drive.getRotation().minus(Rotation2d.kZero).getDegrees()) < Math.abs(drive.getRotation().minus(Rotation2d.k180deg).getDegrees())) {
      drive.requestAutoRotateMode(Rotation2d.kZero);
      superstructure.requestAlgaePrescore(Robot.alliance == DriverStation.Alliance.Red);
    }
    else {
      drive.requestAutoRotateMode(Rotation2d.k180deg);
      superstructure.requestAlgaePrescore(Robot.alliance == DriverStation.Alliance.Blue);
    }
  }

  @Override
  public void execute() {
    if (RobotContainer.isScoringTriggerHeld()) {
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
