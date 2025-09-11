package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.OperationMode;

public class SwitchOperationModeCommand extends Command {

  private Superstructure.Level level;
  private Superstructure superstructure;

  public SwitchOperationModeCommand(Superstructure superstructure) {
    this.superstructure = superstructure;
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
        && !driver.back().getAsBoolean()) {
      if (superstructure.isAutoOperationMode()) {
        Superstructure.OperationMode mode = OperationMode.MANUAL;
      } else {
        Superstructure.OperationMode mode = OperationMode.AUTO;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false; // TODO have actual logic
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
  }
}
