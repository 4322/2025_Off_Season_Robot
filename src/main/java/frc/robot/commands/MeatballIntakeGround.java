package frc.robot.commands;

import static frc.robot.RobotContainer.drivePanr;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class MeatballIntakeGround extends Command {
  private Superstructure superstructure;

  public MeatballIntakeGround(Superstructure superstructure) {
    this.superstructure = superstructure;

    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.requestIntakeMeatballFloor();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return !drivePanr.a().getAsBoolean() || superstructure.isMeatballHeld();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
  }
}
