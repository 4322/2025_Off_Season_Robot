package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;

public class EjectAuto extends Command {
  private IntakeSuperstructure intakeSuperstructure;
  private Superstructure superstructure;

  private EjectType ejectType;
  private double timeout;

  private Timer timer;

  public enum EjectType {
    INTAKE,
    END_EFFECTOR
  }

  public EjectAuto(IntakeSuperstructure intakeSuperstructure, Superstructure superstructure, EjectType ejectType, double timeout) {
    this.intakeSuperstructure = intakeSuperstructure;
    this.superstructure = superstructure;
    this.ejectType = ejectType;
    this.timeout = timeout;

    timer = new Timer();
    timer.reset();
    timer.stop();

    addRequirements(intakeSuperstructure, superstructure);

    
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    if (ejectType == EjectType.INTAKE) {
      intakeSuperstructure.requestEject();
    } else if (ejectType == EjectType.END_EFFECTOR) {
      superstructure.requestEject();
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeout);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.requestIdle();
    intakeSuperstructure.requestRetractIdle();
    timer.stop();
    timer.reset();
  }
}
