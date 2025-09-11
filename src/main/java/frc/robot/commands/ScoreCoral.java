package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.driver;
import frc.robot.subsystems.Superstructure;

public class ScoreCoral extends Command {

  private Superstructure.Level level;
  private Superstructure superstructure;

  public ScoreCoral() {
    this.superstructure = superstructure;
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    superstructure.requestPrescoreCoral(level);
    if (driver.rightTrigger().getAsBoolean() && !superstructure.isAutoOperationMode()) {
      superstructure.requestScoreCoral(level);
    }
    if (!(driver.a().getAsBoolean() || driver.x().getAsBoolean() || driver.y().getAsBoolean() || driver.b().getAsBoolean())) {
      cancel();
      }
  }

  @Override
  public boolean isFinished() {
    return false; // TODO have actual logic
  }

  @Override
  public void end(boolean interrupted) {
    if (!(driver.a().getAsBoolean() || driver.x().getAsBoolean() || driver.y().getAsBoolean() || driver.b().getAsBoolean())) {
    superstructure.requestIdle();
    }
  }
}
