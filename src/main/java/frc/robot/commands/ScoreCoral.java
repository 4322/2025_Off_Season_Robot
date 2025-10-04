package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class ScoreCoral extends Command {

  private Superstructure.Level Level;
  private final Superstructure superstructure;
  private Timer L1OverrideTimer = new Timer();
  private final Vision vision;
  private final Drive drive;

  public ScoreCoral(
      Superstructure superstructure, Superstructure.Level Level, Vision vision, Drive drive) {
    this.superstructure = superstructure;
    this.Level = Level;
    this.vision = vision;
    this.drive = drive;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    L1OverrideTimer.stop();
    L1OverrideTimer.reset();
    superstructure.requestPrescoreCoral(Level);
    drive.requestAutoRotateMode(vision.reefFace);
  }

  @Override
  public void execute() {

    if (RobotContainer.isScoringTriggerHeld()) {
      superstructure.requestScoreCoral(Level);
    }
  }

  @Override
  public boolean isFinished() {
    return !driver.a().getAsBoolean()
        && !driver.x().getAsBoolean()
        && !driver.y().getAsBoolean()
        && !driver.b().getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
  }
}
