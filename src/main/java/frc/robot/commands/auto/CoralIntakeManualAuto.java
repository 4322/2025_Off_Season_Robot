package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSuperstructure;

public class CoralIntakeManualAuto extends Command {
  private IntakeSuperstructure intakeSuperstructure;
  private boolean isAutoEnd;

  public CoralIntakeManualAuto(IntakeSuperstructure intakeSuperstructure, boolean isAutoEnd) {
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
    return (intakeSuperstructure.isCoralDetectedPickupArea() && isAutoEnd)
        || !DriverStation.isAutonomousEnabled();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSuperstructure.requestRetractIdle();
  }
}
