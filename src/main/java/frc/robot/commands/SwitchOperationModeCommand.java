package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class SwitchOperationModeCommand extends Command {

  private Superstructure.OperationMode mode;
  private Superstructure superstructure;

  public SwitchOperationModeCommand(
      Superstructure superstructure, Superstructure.OperationMode mode) {
    this.superstructure = superstructure;
    this.mode = mode;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!driver.a().getAsBoolean()
        && !driver.x().getAsBoolean()
        && !driver.y().getAsBoolean()
        && !driver.b().getAsBoolean()
        && !driver.rightTrigger().getAsBoolean()
        && !driver.leftBumper().getAsBoolean()
        && !driver.rightBumper().getAsBoolean()
        && !driver.leftTrigger().getAsBoolean()
        && !driver.povUp().getAsBoolean()
        && !driver.povDown().getAsBoolean()
        && !driver.povLeft().getAsBoolean()
        && !driver.povRight().getAsBoolean()
        && !driver.start().getAsBoolean()
        && !driver.back().getAsBoolean()
        && driver.leftStick().getAsBoolean()) {
      if (superstructure.isAutoOperationMode()) {
        superstructure.requestOperationMode(Superstructure.OperationMode.MANUAL);
      } else {
        superstructure.requestOperationMode(Superstructure.OperationMode.AUTO);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
