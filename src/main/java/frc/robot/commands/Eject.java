package frc.robot.commands;

import static frc.robot.RobotContainer.drivePanr;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivePan.DrivePan;

public class Eject extends Command {
  private IntakeSuperstructure intakeSuperstructure;
  private Superstructure superstructure;
  private DrivePan drivePan;

  public Eject(
      IntakeSuperstructure intakeSuperstructure, Superstructure superstructure, DrivePan drivePan) {
    this.intakeSuperstructure = intakeSuperstructure;
    this.superstructure = superstructure;
    this.drivePan = drivePan;

    addRequirements(intakeSuperstructure, superstructure);
  }

  @Override
  public void initialize() {
    if (drivePan.getCurrentCommand() != null) {
      if (drivePan.getCurrentCommand() != drivePan.getDefaultCommand()) {
        drivePan.getCurrentCommand().cancel();
      }
    }
  }

  @Override
  public void execute() {

    if (drivePanr.povUp().getAsBoolean()) {
      intakeSuperstructure.requestEject();
    } else if (drivePanr.povDown().getAsBoolean()) {
      superstructure.requestEject();
    }
  }

  @Override
  public boolean isFinished() {
    return !drivePanr.povUp().getAsBoolean() && !drivePanr.povDown().getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
    intakeSuperstructure.requestRetractIdle();
  }
}
