package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;

public class Eject extends Command {
  private IntakeSuperstructure intakeSuperstructure;
  private Superstructure superstructure;

  public Eject(IntakeSuperstructure intakeSuperstructure, Superstructure superstructure) {
    this.intakeSuperstructure = intakeSuperstructure;
    this.superstructure = superstructure;

    addRequirements(intakeSuperstructure, superstructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    superstructure.requestEject();
    intakeSuperstructure.requestEject();
  }

  @Override
  public boolean isFinished() {
    return false; // While held
    // TODO should we automatically cancel this when no piece is detected?
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
    intakeSuperstructure.requestRetractIdle();
  }
}
