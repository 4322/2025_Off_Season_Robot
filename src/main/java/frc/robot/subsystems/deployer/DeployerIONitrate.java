package frc.robot.subsystems.deployer;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.OutputSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import com.reduxrobotics.motorcontrol.requests.PIDPositionRequest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
// TODO add constant for offset pid 
// Fully deployed is 0
// Home is what fully deployed used to be
// Idle retract is home position - before
// Verify that all constants are being used
// Enable GravitationalFeedForward mode
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
    initMotorConfig();
    NitrateSettings deployerMotorConfigStatus =
        deployerMotor.setSettings(deployerMotorConfig, 0.02, 5);
    if (!deployerMotorConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + deployerMotor.getAddress().getDeviceId()
              + " error (Deployer Motor); Did not receive settings",
          false);
    }
  }

  private void initMotorConfig() {
    deployerMotorConfig.getElectricalLimitSettings()
    .setBusCurrentLimit(Constants.Deployer.motorBusCurrentLimit)
    .setBusCurrentLimitTime(Constants.Deployer.motorBusCurrentLimitTime)
        .setStatorCurrentLimit(Constants.Deployer.motorStatorCurrentLimit);

    deployerMotorConfig.getOutputSettings()
        .setIdleMode(Constants.Deployer.motorIdleMode)
        .setInvert(Constants.Deployer.motorInvertMode);

    // Deploy PID has a kG value and is in slot 0
    // Retract PID does not have a kG value and is in slot 1

    deployerMotorConfig.getPIDSettings(PIDConfigSlot.kSlot0)
        .setPID(
            Constants.Deployer.motorDeploykP,
            Constants.Deployer.motorDeploykI,
            Constants.Deployer.motorDeploykD)
        .setGravitationalFeedforward(Constants.Deployer.motorDeployGravitationalFeedforward)
        .setFeedforwardMode(Constants.Deployer.motorFeedforwardMode);

    deployerMotorConfig.getPIDSettings(PIDConfigSlot.kSlot1)
        .setPID(
            Constants.Deployer.motorRetractkP,
            Constants.Deployer.motorRetractkI,
            Constants.Deployer.motorRetractkD)
        .setFeedforwardMode(Constants.Deployer.motorFeedforwardMode);

  }

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    inputs.deployerMotorConnected = deployerMotor.isConnected();
    inputs.deployerMotorStatorCurrentAmps = deployerMotor.getStatorCurrent();
    inputs.deployerMotorBusCurrentAmps = deployerMotor.getBusCurrent();
    inputs.deployerMotorTempCelcius = deployerMotor.getMotorTemperatureFrame().getValue();
    inputs.deployerMotorPositionRotations = Units.degreesToRotations(Constants.Deployer.motorOffsetDegrees) - deployerMotor.getPosition();
    inputs.deployerMotorSpeedRotationsPerSec = deployerMotor.getVelocity();
    inputs.deployerMotorAppliedVolts = deployerMotor.getBusVoltageFrame().getValue();

    inputs.deployerMotorRequestedPositionRotations = deployerMotorRequestedPositionRotations;
  }

  @Override
  public void setDeployerMotorPosition(double rotations) {
    if (rotations != previousRequestedPosition) {
      deployerMotorRequestedPositionRotations = Units.degreesToRotations(Constants.Deployer.motorOffsetDegrees) - (rotations * Constants.Deployer.motorGearRatio);
      previousRequestedPosition = rotations;
      if ((Constants.Deployer.motorOffsetDegrees - deployerMotor.getPosition()) < deployerMotorRequestedPositionRotations) {
        deployerMotor.setRequest(
            deployerMotorDeployPIDRequest.setPosition(Units.degreesToRotations(Constants.Deployer.motorOffsetDegrees) - deployerMotorRequestedPositionRotations));
      } else {
        deployerMotor.setRequest(
            deployerMotorRetractPIDRequest.setPosition(Units.degreesToRotations(Constants.Deployer.motorOffsetDegrees) - deployerMotorRequestedPositionRotations));
      }
    }
  }

  @Override
  public void stopDeployerMotor(IdleMode idleMode) {
    previousRequestedPosition = -999;
    deployerMotor.stop(idleMode);
  }

  @Override
  public void deployerMotorEncoderSetHome() {
    deployerMotor.setPosition(Units.degreesToRotations(Constants.Deployer.motorOffsetDegrees));
  }
}
