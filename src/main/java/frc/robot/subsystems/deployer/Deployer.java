package frc.robot.subsystems.deployer;

import org.littletonrobotics.junction.Logger;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

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

  public Deployer(DeployerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Deployer", inputs);
    Logger.recordOutput("Deployer/currentAction", currentAction.toString());
  }

  public void deploy() {
    currentAction = DeployerStatus.DEPLOY;
    io.setDeployerMotorPosition(Constants.Deployer.deployPositionRotations);
  }

  public void retract() {
    currentAction = DeployerStatus.RETRACT;
    io.setDeployerMotorPosition(Constants.Deployer.retractPositionRotations);
  }

  public void ejectPosition() {
    currentAction = DeployerStatus.EJECT;
    io.setDeployerMotorPosition(Constants.Deployer.ejectPositionRotations);
  }

  public void setHome() {
    io.deployerMotorEncoderSetHome();
  }

  public void setNeutralMode(IdleMode mode) {
    io.stopDeployerMotor(mode);
  }
}
