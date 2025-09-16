package frc.robot.commands;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSuperstructure;

public class CoralIntakeManual extends Command {
  private IntakeSuperstructure intakeSuperstructure;
  private Timer rumbleTimer = new Timer();

  public CoralIntakeManual(IntakeSuperstructure intakeSuperstructure) {
    this.intakeSuperstructure = intakeSuperstructure;
    addRequirements(intakeSuperstructure);
    rumbleTimer.reset();
    rumbleTimer.stop();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSuperstructure.requestIntake();
    if (intakeSuperstructure.isCoralDetectedIntake()) {
      if (!rumbleTimer.isRunning()) {
        rumbleTimer.reset();
        rumbleTimer.start();
        driver.setRumble(RumbleType.kBothRumble, 1);
      } else if (rumbleTimer.hasElapsed(0.25)) {
        driver.setRumble(RumbleType.kBothRumble, 0);
        rumbleTimer.stop();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return !(driver.getLeftTriggerAxis() > 0.5);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSuperstructure.requestRetractIdle();
  }
}
