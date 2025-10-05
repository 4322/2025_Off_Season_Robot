package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class AlgaeIntakeGround extends Command {
  private Superstructure superstructure;

  public AlgaeIntakeGround(Superstructure superstructure) {
    this.superstructure = superstructure;

    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.requestIntakeAlgaeFloor();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return !driver.a().getAsBoolean() || superstructure.isAlgaeHeld();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
  }
}
