package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class DescoreAlgae extends Command {

  private Superstructure.Level Level;
  private final Superstructure superstructure;
  public boolean isSlow = false;

  public DescoreAlgae(Superstructure superstructure, Superstructure.Level Level) {
    this.superstructure = superstructure;
    this.Level = Level;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.requestDescoreAlgae(Level);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return (!driver.x().getAsBoolean()
            && !driver.y().getAsBoolean()
            && superstructure.isAlgaeHeld()
            && !superstructure.isAutoOperationMode())
        || (superstructure.isAutoOperationMode()
            && superstructure.isAlgaeHeld() /*Add drive safeback */);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
    isSlow = false;
  }
}
