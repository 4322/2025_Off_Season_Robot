package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

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
  public void end(boolean interrupted) {
    superstructure.requestIdle();
    intakeSuperstructure.requestRetractIdle();
  }

  @Override
  public boolean isFinished() {
    return !driver.povUp().getAsBoolean();
  }
}
