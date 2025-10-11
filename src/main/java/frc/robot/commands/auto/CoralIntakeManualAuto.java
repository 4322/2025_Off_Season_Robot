package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSuperstructure;

public class CoralIntakeManualAuto extends Command {
  private IntakeSuperstructure intakeSuperstructure;

  public CoralIntakeManualAuto(IntakeSuperstructure intakeSuperstructure) {
    this.intakeSuperstructure = intakeSuperstructure;
    addRequirements(intakeSuperstructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSuperstructure.requestIntake();
    }

  @Override
  public boolean isFinished() {
    return intakeSuperstructure.isCoralDetectedIndexer();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSuperstructure.requestRetractIdle();
  }
}
