package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreCoral extends Command {
  private Arm arm;
  private Elevator elevator;
  private Superstructure.Level level;
  private Superstructure superstructure;

  public ScoreCoral() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (superstructure.getState() == Superstructure.Superstates.PRESCORE_CORAL) {
      superstructure.getReefStatus();
      if (superstructure.isAutoOperationMode()) {
      } else {

      }
    }

    arm.scoreCoral(level);
    elevator.scoreCoral(level);
  }

  @Override
  public boolean isFinished() {
    return false; // TODO have actual logic
  }

  @Override
  public void end(boolean interrupted) {}
}
