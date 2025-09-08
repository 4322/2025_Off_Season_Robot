package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;

public class ScoreCoral extends Command {
  private Arm arm;
  private Elevator elevator;
  private Superstructure.Level level;
  private Superstructure superstructure;
  private EndEffector endEffector;

  public boolean isFast = true;

  public ScoreCoral() {
    this.superstructure = superstructure;
    this.arm = arm;
    this.elevator = elevator;
    this.endEffector = endEffector;
  }

  @Override
  public void initialize() {
    isFast = false;
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false; // TODO have actual logic
  }

  @Override
  public void end(boolean interrupted) {}
}
