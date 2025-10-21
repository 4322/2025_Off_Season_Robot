package frc.robot.commands;

import static frc.robot.RobotContainer.drivePanr;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;

public class SwitchOperationModeCommand extends InstantCommand {

  private Superstructure superstructure;

  public SwitchOperationModeCommand(Superstructure superstructure) {
    this.superstructure = superstructure;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    if (!drivePanr.a().getAsBoolean()
        && !drivePanr.x().getAsBoolean()
        && !drivePanr.y().getAsBoolean()
        && !drivePanr.b().getAsBoolean()
        && !RobotContainer.isScoringTriggerHeld()
        && !drivePanr.leftBumper().getAsBoolean()
        && !drivePanr.rightBumper().getAsBoolean()
        && !drivePanr.leftTrigger().getAsBoolean()
        && !drivePanr.povUp().getAsBoolean()
        && !drivePanr.povDown().getAsBoolean()
        && !drivePanr.povLeft().getAsBoolean()
        && !drivePanr.povRight().getAsBoolean()
        && !drivePanr.start().getAsBoolean()
        && !drivePanr.back().getAsBoolean()) {
      if (superstructure.isAutoOperationMode()) {
        superstructure.requestOperationMode(Superstructure.OperationMode.MANUAL);
      } else {
        superstructure.requestOperationMode(Superstructure.OperationMode.TeleAUTO);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}
}
