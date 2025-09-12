package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.driver;
import frc.robot.subsystems.Superstructure;

public class ScoreCoral extends Command {

  private Superstructure.Level level;
  private Superstructure superstructure;
  public boolean isSlow = false;

  public ScoreCoral(Superstructure superstructure, Superstructure.Level level) {
    this.superstructure = superstructure;
    this.level = level;

    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.requestPrescoreCoral(level);
  }

  @Override
  public void execute() {
    isSlow = true;
    if (driver.rightTrigger().getAsBoolean() && !superstructure.isAutoOperationMode()) {
      superstructure.requestScoreCoral(level);
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
