package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class AlgaeScoreCommand extends Command {

  private Superstructure.Level level;
  private Superstructure superstructure;

  public AlgaeScoreCommand(Superstructure superstructure) {
    this.superstructure = superstructure;

    addRequirements(superstructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (driver.rightTrigger().getAsBoolean() || superstructure.isAlgaeHeld()) {
      superstructure.requestAlgaeScore();
    } else {
      superstructure.requestAlgaePrescore();
    }

    if ((!driver.b().getAsBoolean() && !superstructure.isAutoOperationMode())
        || (!superstructure.isAlgaeHeld() && superstructure.isAutoOperationMode())) {
      cancel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
  }
}
