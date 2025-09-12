package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.driver;
import frc.robot.subsystems.Superstructure;

public class ScoreCoral extends Command {

  private Superstructure.Level Level;
  private Superstructure superstructure;
  public boolean isSlow = false;

  public ScoreCoral(Superstructure superstructure, Superstructure.Level Level) {
    this.superstructure = superstructure;
    this.Level = Level;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.requestPrescoreCoral(Level);
  }

  @Override
  public void execute() {
    isSlow = true;
    if (driver.rightTrigger().getAsBoolean()) {
      superstructure.requestScoreCoral(Level);
    }
  }

  @Override
  public boolean isFinished() {
    return (!driver.a().getAsBoolean()
            && !driver.x().getAsBoolean()
            && !driver.y().getAsBoolean()
            && !driver.b().getAsBoolean())
        || !superstructure.isCoralHeld();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
    isSlow = false;
  }
}
