package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.driver;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class DescoreAlgae extends Command {

  private Superstructure.Level Level;
  private final Superstructure superstructure;
  private final Vision vision;
  private final Drive drive;

  public DescoreAlgae(
      Superstructure superstructure, Superstructure.Level Level, Drive drive, Vision vision) {
    this.superstructure = superstructure;
    this.Level = Level;
    this.vision = vision;
    this.drive = drive;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    // ReefStatus reefStatus = vision.getReefStatus();
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
  }
}
