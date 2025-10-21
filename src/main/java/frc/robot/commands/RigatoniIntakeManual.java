package frc.robot.commands;

import static frc.robot.RobotContainer.drivePanr;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSuperstructure;

public class RigatoniIntakeManual extends Command {
  private IntakeSuperstructure intakeSuperstructure;
  private Timer rumbleTimer = new Timer();
  private boolean autoEnd;

  public RigatoniIntakeManual(IntakeSuperstructure intakeSuperstructure, boolean autoEnd) {
    this.intakeSuperstructure = intakeSuperstructure;
    addRequirements(intakeSuperstructure);
    rumbleTimer.reset();
    rumbleTimer.stop();
    this.autoEnd = autoEnd;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSuperstructure.requestIntake();
    if (intakeSuperstructure.isRigatoniDetectedIntake()) {
      if (!rumbleTimer.isRunning()) {
        rumbleTimer.reset();
        rumbleTimer.start();
        drivePanr.setRumble(RumbleType.kBothRumble, 1);
      } else if (rumbleTimer.hasElapsed(0.25)) {
        drivePanr.setRumble(RumbleType.kBothRumble, 0);
        rumbleTimer.stop();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return !(drivePanr.getLeftTriggerAxis() > 0.5)
        || (autoEnd && intakeSuperstructure.isRigatoniDetectedPastaWheels());
  }

  @Override
  public void end(boolean interrupted) {
    intakeSuperstructure.requestRetractIdle();
  }
}
