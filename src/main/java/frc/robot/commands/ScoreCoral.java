package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class ScoreCoral extends Command {

  private Superstructure.Level level;
  private Superstructure superstructure;

  public ScoreCoral() {
    this.superstructure = superstructure;
  }

  @Override
  public void initialize() {
    superstructure.requestPrescoreCoral(level);
  }

  @Override
  public void execute() {
    superstructure.requestScoreCoral(level);
  }

  @Override
  public boolean isFinished() {
    return false; // TODO have actual logic
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
  }
}
