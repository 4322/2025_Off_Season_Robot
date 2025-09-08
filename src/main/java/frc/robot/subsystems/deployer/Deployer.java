package frc.robot.subsystems.deployer;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Deployer extends SubsystemBase {
  private DeployerIO io;
  private DeployerIOInputsAutoLogged inputs = new DeployerIOInputsAutoLogged();

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
  }

  public void deploy() {
    if (!isHomed || currentAction == DeployerStatus.DEPLOY) {
      return;
    }
    currentAction = DeployerStatus.DEPLOY;
    io.setDeployerMotorPosition(Constants.Deployer.deployPositionRotations);
  }

  public void retract() {
    if (!isHomed || currentAction == DeployerStatus.RETRACT) {
      return;
    }
    currentAction = DeployerStatus.RETRACT;
    io.setDeployerMotorPosition(Constants.Deployer.retractPositionRotations);
  }

  public void eject() {
    if (!isHomed || currentAction == DeployerStatus.EJECT) {
      return;
    }
    currentAction = DeployerStatus.EJECT;
    io.setDeployerMotorPosition(Constants.Deployer.ejectPositionRotations);
  }

  public void setHome() {
    io.deployerMotorEncoderSetHome();
    isHomed = true;
  }

  public void clearHome() {
    isHomed = false;
  }

  public void setNeutralMode(IdleMode mode) {
    io.stopDeployerMotor(mode);
  }
}
