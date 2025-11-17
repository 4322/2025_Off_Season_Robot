package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class SafeReefRetract extends Command {
  private final Superstructure superstructure;
  private ScoreCoral scoreCoral;

  public SafeReefRetract(Superstructure superstructure) {
    this.superstructure = superstructure;
    addRequirements(superstructure);
  }

  @Override
  public boolean isFinished() {
    return scoreCoral.isInSafeArea();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
  }
}
