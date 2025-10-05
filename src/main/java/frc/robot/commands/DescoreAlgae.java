package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class DescoreAlgae extends Command {

  private Superstructure.Level Level;
  private final Superstructure superstructure;
  private final Drive drive;

  public DescoreAlgae(Superstructure superstructure, Superstructure.Level level, Drive drive) {
    this.superstructure = superstructure;
    this.Level = level;
    this.drive = drive;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    // ReefStatus reefStatus = superstructure.getReefStatus();
    // new DriveToPose(drive, new Pose2d(new Translation2d(),
    // reefStatus.getClosestReefFaceAngle()));
    superstructure.requestDescoreAlgae(Level);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return (!driver.x().getAsBoolean() && !driver.y().getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
    drive.requestFieldRelativeMode();
  }
}
