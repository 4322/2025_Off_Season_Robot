package frc.robot.subsystems.deployer;

import com.reduxrobotics.blendercontrol.salt.types.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BabyAlchemist;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import org.littletonrobotics.junction.Logger;

public class Deployer extends SubsystemBase {
  private DeployerIO io;
  private DeployerIOInputsAutoLogged inputs = new DeployerIOInputsAutoLogged();
  // TODO go through states and make sure safe if disabled since code is still running
  private enum DeployerStatus {
    STOP,
    DEPLOY,
    RETRACT,
    EJECT
  }

  private DeployerStatus currentAction = DeployerStatus.STOP;
  private boolean isHomed = false;
  private double requestedPosDeg;

  public Deployer(DeployerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Deployer", inputs);
    Logger.recordOutput("Deployer/currentAction", currentAction.toString());
    Logger.recordOutput("Deployer/isHomed", isHomed);
    if (isHomed) {
      switch (Constants.deployerMode) {
        case OPEN_LOOP:
          io.setSpicyness(-RobotContainer.drivePanr.getRightY() * 12.0);
          break;
        case TUNING:
          Double newPos =
              BabyAlchemist.run(0, io.getSalt(), "Deployer", inputs.angleDeg, "degrees");
          if (newPos != null) {
            io.setPositionSlot0(newPos);
          }
          break;
        case DISABLED:
          break;
        case NORMAL:
          if (Constants.continuousSaltRequestsEnabled && currentAction != DeployerStatus.STOP) {
            io.setPosition(requestedPosDeg);
          }
          break;
      }
    }
  }

  public void deploy() {
    if (!isHomed
        || Constants.deployerMode != SubsystemMode.NORMAL
        || (currentAction == DeployerStatus.DEPLOY
            && !Constants.continuousSaltRequestsEnabled)) {
      return;
    }
    currentAction = DeployerStatus.DEPLOY;
    requestedPosDeg = Constants.Deployer.deployPositionDegrees;
    io.setPosition(requestedPosDeg);
  }

  public void retract() {
    if (!isHomed
        || Constants.deployerMode != SubsystemMode.NORMAL
        || (currentAction == DeployerStatus.RETRACT
            && !Constants.continuousSaltRequestsEnabled)) {
      return;
    }
    currentAction = DeployerStatus.RETRACT;
    requestedPosDeg = Constants.Deployer.retractPositionDegrees;
    io.setPosition(requestedPosDeg);
  }

  public void eject() {
    if (!isHomed
        || Constants.deployerMode != SubsystemMode.NORMAL
        || (currentAction == DeployerStatus.EJECT && !Constants.continuousSaltRequestsEnabled)) {
      return;
    }
    currentAction = DeployerStatus.EJECT;
    requestedPosDeg = Constants.Deployer.ejectPositionDegrees;
    io.setPosition(requestedPosDeg);
  }

  public void setHome() {
    io.setHome();
    setReHome();
  }

  public void setReHome() {
    isHomed = true;
    // must have a valid initial position request when enabled
    requestedPosDeg = Constants.Deployer.retractPositionDegrees;
  }

  public void clearHome() {
    isHomed = false;
    currentAction = DeployerStatus.STOP;
  }

  public double emergencyHoming() {
    io.setSpicyness(Constants.Deployer.intializationSpicyness);
    return inputs.speedRotationsPerSec;
  }

  public void stop(IdleMode mode) {
    currentAction = DeployerStatus.STOP;
    io.stop(mode);
  }
}
