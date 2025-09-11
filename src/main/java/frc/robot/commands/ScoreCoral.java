package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.driver;
import frc.robot.subsystems.Superstructure;

public class ScoreCoral extends Command {

  private Superstructure.Level level;
  private Superstructure superstructure;

  public ScoreCoral(Superstructure superstructure) {
    this.superstructure = superstructure;

    addRequirements(superstructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (driver.rightTrigger().getAsBoolean() && !superstructure.isAutoOperationMode()) {
      superstructure.requestScoreCoral(level);
    } else {
      superstructure.requestPrescoreCoral(level);
    }

    if ((!driver.a().getAsBoolean()
        && !driver.x().getAsBoolean()
        && !driver.y().getAsBoolean()
        && !driver.b().getAsBoolean()) || superstructure.isCoralHeld()) {
      cancel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
  }
}
