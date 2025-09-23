package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;

public class ScoreCoral extends Command {

  private Superstructure.Level Level;
  private final Superstructure superstructure;

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
    double x = -RobotContainer.driver.getLeftY();
    double y = -RobotContainer.driver.getLeftX();
    if (RobotContainer.isScoringTriggerHeld()) {
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
  }
}
