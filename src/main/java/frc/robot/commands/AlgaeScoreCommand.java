package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;

public class AlgaeScoreCommand extends Command {
  private Superstructure superstructure;

  public AlgaeScoreCommand(Superstructure superstructure) {
    this.superstructure = superstructure;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.requestAlgaePrescore();
  }

  @Override
  public void execute() {
    if (RobotContainer.isScoringTriggerHeld() && superstructure.isAlgaeHeld()) {
      superstructure.requestAlgaeScore();
    }
  }

  @Override
  public boolean isFinished() {
    return !driver.b().getAsBoolean()
        || (!superstructure.isAlgaeHeld() && superstructure.isAutoOperationMode());
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
  }
}
