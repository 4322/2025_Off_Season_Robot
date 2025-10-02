package frc.robot.subsystems.deployer;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
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
    START,
    DEPLOY,
    RETRACT,
    EJECT
  }

  private DeployerStatus currentAction = DeployerStatus.START;
  private boolean isHomed = false;

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
          io.setVoltage(-RobotContainer.driver.getRightY() * 12.0);
          break;
        case TUNING:
          Double newPos =
              BabyAlchemist.run(0, io.getNitrate(), "Deployer", inputs.angleDeg, "degrees");
          if (newPos != null) {
            io.setPositionSlot0(newPos);
          }
          break;
        case DISABLED:
          break;
        case NORMAL:
          break;
      }
    }
  }

  public void deploy() {
    if (!isHomed
        || Constants.deployerMode != SubsystemMode.NORMAL
        || currentAction == DeployerStatus.DEPLOY) {
      return;
    }
    currentAction = DeployerStatus.DEPLOY;
    io.setPosition(Constants.Deployer.deployPositionDegrees);
  }

  public void retract() {
    if (!isHomed
        || Constants.deployerMode != SubsystemMode.NORMAL
        || currentAction == DeployerStatus.RETRACT) {
      return;
    }
    currentAction = DeployerStatus.RETRACT;
    io.setPosition(Constants.Deployer.retractPositionDegrees);
  }

  public void eject() {
    if (!isHomed
        || Constants.deployerMode != SubsystemMode.NORMAL
        || currentAction == DeployerStatus.EJECT) {
      return;
    }
    currentAction = DeployerStatus.EJECT;
    io.setPosition(Constants.Deployer.ejectPositionDegrees);
  }

  public void setHome() {
    io.setHome();
    isHomed = true;
  }

  public void clearHome() {
    isHomed = false;
  }

  public void stop(IdleMode mode) {
    io.stop(mode);
  }

  public boolean isDeployed() {
    return currentAction == DeployerStatus.DEPLOY;
  }
}
