package frc.robot.subsystems.deployer;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.FeedbackSensorSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.FramePeriodSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.OutputSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.EnabledDebugFrames;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MinwrapConfig;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import com.reduxrobotics.motorcontrol.requests.PIDPositionRequest;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

/* Code coordinate system:
 * 0 -> 145.353984 where 0 is fully deployed and 145.353984 is fully retracted against hardstop
 * Motor controller coordinate system:
 * -40 -> 0 -> 135.353984 where 135.353984 is fully retracted against hardstop,
 * 0 is the the point where the deployer is the most affected by gravity, and -40 is fully deployed
 *
 * code = motor + 40
 * motor = code - 40
 */

public class DeployerIONitrate implements DeployerIO {
  private Nitrate deployerMotor;

  private NitrateSettings motorConfig = new NitrateSettings();

  private final PIDPositionRequest deployerMotorDeployPIDRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);
  private final PIDPositionRequest deployerMotorRetractPIDRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot1, 0).useMotionProfile(true);

  private double requestedPosDeg = -1;
  private double mechanismAngleDeg;

  public DeployerIONitrate() {
    deployerMotor = new Nitrate(Constants.Deployer.deployerMotorId, MotorType.kCu60);
    initMotorConfig();
    NitrateSettings deployerMotorConfigStatus = deployerMotor.setSettings(motorConfig, 0.1, 5);
    if (!deployerMotorConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + deployerMotor.getAddress().getDeviceId()
              + " error (Deployer Motor); Did not receive settings",
          false);
    }
  }

  private void initMotorConfig() {
    motorConfig.setElectricalLimitSettings(
        ElectricalLimitSettings.defaultSettings()
            .setBusCurrentLimit(Constants.Deployer.busCurrentLimit)
            .setBusCurrentLimitTime(Constants.Deployer.busCurrentLimitTime)
            .setStatorCurrentLimit(Constants.Deployer.statorCurrentLimit));

    motorConfig.setOutputSettings(
        OutputSettings.defaultSettings()
            .setIdleMode(Constants.Deployer.idleMode)
            .setInvert(Constants.Deployer.invertMode));

    motorConfig.setFeedbackSensorSettings(
        FeedbackSensorSettings.defaultSettings()
            .setSensorToMechanismRatio(Constants.Deployer.motorGearRatio));

    // Deploy PID in slot 0
    // Retract PID in slot 1

    motorConfig.setPIDSettings(
        PIDSettings.defaultSettings(PIDConfigSlot.kSlot0)
            .setPID(
                Constants.Deployer.deploykP,
                Constants.Deployer.deploykI,
                Constants.Deployer.deploykD)
            .setGravitationalFeedforward(Constants.Deployer.kG)
            .setFeedforwardMode(Constants.Deployer.feedforwardMode)
            .setMinwrapConfig(new MinwrapConfig.Disabled())
            .setMotionProfileAccelLimit(Constants.Deployer.accelerationLimit)
            .setMotionProfileDeaccelLimit(Constants.Deployer.deaccelerationLimit)
            .setMotionProfileVelocityLimit(Constants.Deployer.velocityLimit)
            .setISaturation(Constants.Deployer.iSat)
            .setIZone(Constants.Deployer.iZone)
            .setRampLimit(240),
        PIDConfigSlot.kSlot0);

    motorConfig.setPIDSettings(
        PIDSettings.defaultSettings(PIDConfigSlot.kSlot1)
            .setPID(Constants.Deployer.retractkP, 0, 0)
            .setGravitationalFeedforward(Constants.Deployer.kG)
            .setFeedforwardMode(Constants.Deployer.feedforwardMode)
            .setMinwrapConfig(new MinwrapConfig.Disabled())
            .setMotionProfileAccelLimit(Constants.Deployer.accelerationLimit)
            .setMotionProfileDeaccelLimit(Constants.Deployer.deaccelerationLimit)
            .setMotionProfileVelocityLimit(Constants.Deployer.velocityLimit)
            .setISaturation(Constants.Deployer.iSat)
            .setIZone(Constants.Deployer.iZone)
            .setRampLimit(240),
        PIDConfigSlot.kSlot1);

    motorConfig.setFramePeriodSettings(
        FramePeriodSettings.defaultSettings()
            .setEnabledPIDDebugFrames(
                new EnabledDebugFrames()
                    .setKgControlEffort(Constants.debugPIDModeEnabled)
                    .setKpControlEffort(Constants.debugPIDModeEnabled)
                    .setTotalControlEffort(Constants.debugPIDModeEnabled)));
  }

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    inputs.connected = deployerMotor.isConnected();
    inputs.statorCurrentAmps = deployerMotor.getStatorCurrent();
    inputs.busCurrentAmps = deployerMotor.getBusCurrent();
    inputs.motorTempCelcius = deployerMotor.getMotorTemperatureFrame().getValue();
    inputs.controllerTempCelcius = deployerMotor.getControllerTemperatureFrame().getValue();
    inputs.angleDeg = toCodeCoords(Units.rotationsToDegrees(deployerMotor.getPosition()));
    inputs.speedRotationsPerSec = deployerMotor.getVelocity();
    inputs.appliedVolts = deployerMotor.getAppliedVoltageFrame().getValue();
    inputs.encoderRotations = deployerMotor.getPosition();
    if (Constants.debugPIDModeEnabled) {
      inputs.kPeffort = deployerMotor.getPIDDebugFrames().kPControlEffortFrame.getValue();
      inputs.kGeffort = deployerMotor.getPIDDebugFrames().kGControlEffortFrame.getValue();
      inputs.totalEffort = deployerMotor.getPIDDebugFrames().totalControlEffortFrame.getValue();
    }

    inputs.requestedPosDeg = requestedPosDeg;

    mechanismAngleDeg = inputs.angleDeg;
  }

  @Override
  public void setPosition(double degrees) {
    // Requested position in code coordinate system
    requestedPosDeg = degrees;

    if ((mechanismAngleDeg) > degrees) {
      deployerMotor.setRequest(
          deployerMotorDeployPIDRequest.setPosition(
              Units.degreesToRotations(toMotorCoords(degrees))));
    } else {
      deployerMotor.setRequest(
          deployerMotorRetractPIDRequest.setPosition(
              Units.degreesToRotations(toMotorCoords(degrees))));
    }
  }

  @Override
  public void setPositionSlot0(double degrees) {
    deployerMotor.setRequest(
        deployerMotorDeployPIDRequest.setPosition(
            Units.degreesToRotations(toMotorCoords(degrees))));
    requestedPosDeg = degrees;
  }

  @Override
  public void setVoltage(double voltage) {
    requestedPosDeg = -1;
    deployerMotor.setVoltage(voltage);
  }

  @Override
  public void stop(IdleMode idleMode) {
    requestedPosDeg = -1;
    deployerMotor.stop(idleMode);
    deployerMotor.setVoltage(0); // work around stop not working
  }

  @Override
  public void setHome() {
    deployerMotor.setPosition(
        Units.degreesToRotations(toMotorCoords(Constants.Deployer.maxRangeDegrees)));
  }

  @Override
  public Nitrate getNitrate() {
    return deployerMotor;
  }

  private double toCodeCoords(double position) {
    return position + Constants.Deployer.maxGravityDegrees;
  }

  private double toMotorCoords(double position) {
    return position - Constants.Deployer.maxGravityDegrees;
  }
}
