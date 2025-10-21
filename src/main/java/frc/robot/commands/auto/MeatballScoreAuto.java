package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivePan.DrivePan;

public class MeatballScoreAuto extends Command {
  private Superstructure superstructure;
  private DrivePan drivePan;

  public MeatballScoreAuto(Superstructure superstructure, DrivePan drivePan) {
    this.superstructure = superstructure;
    this.drivePan = drivePan;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.requestMeatballScore();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return !superstructure.isMeatballHeld();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
    drivePan.requestFieldRelativeMode();
  }
}
