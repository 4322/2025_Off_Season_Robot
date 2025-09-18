package frc.robot.subsystems.deployer;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import com.reduxrobotics.motorcontrol.requests.PIDPositionRequest;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
// TODO add constant for offset pid
// Fully deployed is 0
// Home is what fully deployed used to be
// Idle retract is home position - before
// Verify that all constants are being used
// Enable GravitationalFeedForward mode

/* Code coordinate system:
 * 145.353984 -> 0 where 0 is fully retracted against hardstop and 145.353984 is fully deployed
 * Motor controller coordinate system:
 * -10? -> 0 -> 135.353984 where 135.353984 is fully retracted against hardstop, 0 is the the point where the deployer is the most affected by gravity, and -10? is fully deployed
 * 
 * code = -motor + 135.353984
 * motor = -code + 135.353984
 * 
 */

public class DeployerIONitrate implements DeployerIO {
  private Nitrate deployerMotor;

  private NitrateSettings motorConfig = new NitrateSettings();

  private final PIDPositionRequest deployerMotorDeployPIDRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);
  private final PIDPositionRequest deployerMotorRetractPIDRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot1, 0).useMotionProfile(true);

  private double previousRequestedPosition = -999;
  private double requestedPositionDegrees = 0.0;

  public DeployerIONitrate() {
    deployerMotor = new Nitrate(Constants.Deployer.deployerMotorId, MotorType.kCu60);
    initMotorConfig();
    NitrateSettings deployerMotorConfigStatus =
        deployerMotor.setSettings(motorConfig, 0.02, 5);
    if (!deployerMotorConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + deployerMotor.getAddress().getDeviceId()
              + " error (Deployer Motor); Did not receive settings",
          false);
    }
  }

  private void initMotorConfig() {
    motorConfig
        .getElectricalLimitSettings()
        .setBusCurrentLimit(Constants.Deployer.busCurrentLimit)
        .setBusCurrentLimitTime(Constants.Deployer.busCurrentLimitTime)
        .setStatorCurrentLimit(Constants.Deployer.statorCurrentLimit);

    motorConfig
        .getOutputSettings()
        .setIdleMode(Constants.Deployer.idleMode)
        .setInvert(Constants.Deployer.invertMode);

    // Deploy PID in slot 0
    // Retract PID in slot 1

    motorConfig
        .getPIDSettings(PIDConfigSlot.kSlot0)
        .setPID(
            Constants.Deployer.deploykP,
            Constants.Deployer.deploykI,
            Constants.Deployer.deploykD)
        .setGravitationalFeedforward(Constants.Deployer.deploykG)
        .setFeedforwardMode(Constants.Deployer.feedforwardMode);

    motorConfig
        .getPIDSettings(PIDConfigSlot.kSlot1)
        .setPID(
            Constants.Deployer.retractkP,
            Constants.Deployer.retractkI,
            Constants.Deployer.retractkD)
        .setGravitationalFeedforward(Constants.Deployer.retractkG)
        .setFeedforwardMode(Constants.Deployer.feedforwardMode);
  }

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    inputs.deployerMotorConnected = deployerMotor.isConnected();
    inputs.deployerMotorStatorCurrentAmps = deployerMotor.getStatorCurrent();
    inputs.deployerMotorBusCurrentAmps = deployerMotor.getBusCurrent();
    inputs.deployerMotorTempCelcius = deployerMotor.getMotorTemperatureFrame().getValue();
    inputs.deployerMotorPositionRotations = switchCoordinateSystem(deployerMotor.getPosition());
    inputs.deployerMotorMechanismPositionDegrees = switchCoordinateSystem(inputs.deployerMotorPositionRotations) / Constants.Deployer.motorGearRatio;
    inputs.deployerMotorSpeedRotationsPerSec = deployerMotor.getVelocity();
    inputs.deployerMotorAppliedVolts = deployerMotor.getBusVoltageFrame().getValue();

    inputs.deployerMotorRequestedPositionRotations = requestedPositionDegrees;
  }

  @Override
  public void setDeployerMotorPosition(double degrees) {
    if (degrees != previousRequestedPosition) {
      // Requested position in code coordinate system
      requestedPositionDegrees =
          degrees * Constants.Deployer.motorGearRatio;
      previousRequestedPosition = degrees * Constants.Deployer.motorGearRatio;
      
      if ((switchCoordinateSystem(Units.rotationsToDegrees(deployerMotor.getPosition()) * Constants.Deployer.motorGearRatio))
          < requestedPositionDegrees) {
        deployerMotor.setRequest(
            deployerMotorDeployPIDRequest.setPosition(
                switchCoordinateSystem(requestedPositionDegrees)));
      } else {
        deployerMotor.setRequest(
            deployerMotorRetractPIDRequest.setPosition(
                switchCoordinateSystem(requestedPositionDegrees)));
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
    deployerMotor.setPosition(switchCoordinateSystem(0));
  }

  private double switchCoordinateSystem(double position) {
    return -position + (Constants.Deployer.rangeDegrees - Constants.Deployer.maxGravityDegrees);
  }
}
