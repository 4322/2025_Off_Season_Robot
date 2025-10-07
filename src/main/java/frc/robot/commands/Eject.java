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
    if (driver.povUp().getAsBoolean()) {
      intakeSuperstructure.requestEject();
    } else if (driver.povDown().getAsBoolean()) {
      superstructure.requestEject();
    }
  }

  @Override
  public boolean isFinished() {
    return !driver.povUp().getAsBoolean() && !driver.povDown().getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
    intakeSuperstructure.requestRetractIdle();
  }
}
