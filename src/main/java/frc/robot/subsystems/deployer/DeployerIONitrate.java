package frc.robot.subsystems.deployer;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.OutputSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MotionProfileMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import com.reduxrobotics.motorcontrol.requests.PIDPositionRequest;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class DeployerIONitrate implements DeployerIO {
  private Nitrate deployerMotor;

  private NitrateSettings deployerMotorConfig = new NitrateSettings();

  private final PIDPositionRequest deployerMotorDeployPIDRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);
  private final PIDPositionRequest deployerMotorRetractPIDRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot1, 0).useMotionProfile(true);

  private double previousRequestedPosition = -999;
  private double deployerMotorRequestedPositionRotations = 0.0;

  public DeployerIONitrate() {
    deployerMotor = new Nitrate(Constants.Deployer.deployerMotorId, MotorType.kCu60);
    configMotor();
    NitrateSettings deployerMotorConfigStatus =
        deployerMotor.setSettings(deployerMotorConfig, 0.02, 5);
    if (!deployerMotorConfigStatus.allSettingsReceived()) {
      DriverStation.reportError(
          "Nitrate "
              + deployerMotor.getAddress().getDeviceId()
              + " error (Deployer Motor); Did not receive settings",
          null);
    }
  }

  private void configMotor() {
    // TODO add other settings for motor

    ElectricalLimitSettings deployerMotorElectricalLimitSettings = new ElectricalLimitSettings();
    deployerMotorElectricalLimitSettings.setBusCurrentLimit(
        Constants.Deployer.motorBusCurrentLimit);
    deployerMotorElectricalLimitSettings.setBusCurrentLimitTime(
        Constants.Deployer.motorBusCurrentLimitTime);
    deployerMotorElectricalLimitSettings.setStatorCurrentLimit(
        Constants.Deployer.motorStatorCurrentLimit);
    deployerMotorConfig.setElectricalLimitSettings(deployerMotorElectricalLimitSettings);

    OutputSettings deployerMotorOutputSettings = new OutputSettings();
    deployerMotorOutputSettings.setInvert(Constants.Deployer.motorInvertMode);
    deployerMotorConfig.setOutputSettings(deployerMotorOutputSettings);

    // Deploy PID has a kG value and is in slot 0
    // Retract PID does not have a kG value and is in slot 1
    PIDSettings deployerMotorPIDSettingsDeploy = new PIDSettings();
    deployerMotorPIDSettingsDeploy.setPID(
        Constants.Deployer.motorDeploykP,
        Constants.Deployer.motorDeploykI,
        Constants.Deployer.motorDeploykD);
    deployerMotorPIDSettingsDeploy.setMotionProfileMode(MotionProfileMode.kTrapezoidal);
    deployerMotorPIDSettingsDeploy.setGravitationalFeedforward(
        Constants.Deployer.motorDeployGravitationalFeedforward); // TODO is this kG value?
    deployerMotorConfig.setPIDSettings(deployerMotorPIDSettingsDeploy, PIDConfigSlot.kSlot0);

    PIDSettings deployerMotorPIDSettingsRetract = new PIDSettings();
    deployerMotorPIDSettingsRetract.setPID(
        Constants.Deployer.motorRetractkP,
        Constants.Deployer.motorRetractkI,
        Constants.Deployer.motorRetractkD);
    deployerMotorPIDSettingsRetract.setMotionProfileMode(MotionProfileMode.kTrapezoidal);
    deployerMotorConfig.setPIDSettings(deployerMotorPIDSettingsRetract, PIDConfigSlot.kSlot1);
  }

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    inputs.deployerMotorConnected = deployerMotor.isConnected();
    inputs.deployerMotorStatorCurrentAmps = deployerMotor.getStatorCurrent();
    inputs.deployerMotorBusCurrentAmps = deployerMotor.getBusCurrent();
    inputs.deployerMotorTempCelcius = deployerMotor.getMotorTemperatureFrame().getValue();
    inputs.deployerMotorPositionRotations = deployerMotor.getPosition();
    inputs.deployerMotorSpeedRotationsPerSec = deployerMotor.getVelocity();
    inputs.deployerMotorAppliedVolts = deployerMotor.getBusVoltageFrame().getValue();

    inputs.deployerMotorRequestedPositionRotations = deployerMotorRequestedPositionRotations;
  }

  @Override
  public void setDeployerMotorPosition(double rotations) {
    // TODO This is assuming deploying is positive direction
    if (rotations != previousRequestedPosition) {
      deployerMotorRequestedPositionRotations = rotations * Constants.Deployer.motorGearRatio;
      previousRequestedPosition = rotations;
      if (deployerMotor.getPosition() < rotations * Constants.Deployer.motorGearRatio) {
        deployerMotor.setRequest(
            deployerMotorDeployPIDRequest.setPosition(
                rotations * Constants.Deployer.motorGearRatio));
      } else {
        deployerMotor.setRequest(
            deployerMotorRetractPIDRequest.setPosition(
                rotations * Constants.Deployer.motorGearRatio));
      }
    }
  }

  @Override
  public void stopDeployerMotor(IdleMode idleMode) {
    deployerMotor.stop(idleMode);
  }

  @Override
  public void deployerMotorEncoderSetHome() {
    deployerMotor.setPosition(0);
  }
}
