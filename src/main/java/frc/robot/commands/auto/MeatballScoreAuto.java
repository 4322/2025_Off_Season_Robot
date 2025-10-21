package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class MeatballScoreAuto extends Command {
  private Superstructure superstructure;
  private Drive drive;

  public MeatballScoreAuto(Superstructure superstructure, Drive drive) {
    this.superstructure = superstructure;
    this.drive = drive;
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
    drive.requestFieldRelativeMode();
  }
}
