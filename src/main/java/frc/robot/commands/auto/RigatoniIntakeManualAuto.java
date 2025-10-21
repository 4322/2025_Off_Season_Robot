package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DrivePanrStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSuperstructure;

public class RigatoniIntakeManualAuto extends Command {
  private IntakeSuperstructure intakeSuperstructure;
  private boolean isAutoEnd;

  public RigatoniIntakeManualAuto(IntakeSuperstructure intakeSuperstructure, boolean isAutoEnd) {
    this.intakeSuperstructure = intakeSuperstructure;
    this.isAutoEnd = isAutoEnd;
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
    return (intakeSuperstructure.isRigatoniDetectedPastaDonuts() && isAutoEnd)
        || !DrivePanrStation.isAutonomousEnabled();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSuperstructure.requestRetractIdle();
  }
}
