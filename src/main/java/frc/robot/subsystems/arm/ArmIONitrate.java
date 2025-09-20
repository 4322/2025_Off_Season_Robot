package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.FeedbackSensorSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.FramePeriodSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.OutputSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.EnabledDebugFrames;
import com.reduxrobotics.motorcontrol.nitrate.types.FeedbackSensor;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MinwrapConfig;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDFeedforwardMode;
import com.reduxrobotics.motorcontrol.requests.PIDPositionRequest;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class ArmIONitrate implements ArmIO {

  private final Nitrate armMotor;
  private final Canandmag armEncoder;
  private double lastRequestedPosDeg;

  private final PIDPositionRequest PIDPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);
  private final PIDPositionRequest SlowPIDPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot1, 0).useMotionProfile(true);

  public ArmIONitrate() {
    armMotor = new Nitrate(Constants.Arm.armMotorId, MotorType.kCu60);
    armEncoder = new Canandmag(Constants.Arm.armEncoderId);
    NitrateSettings armConfig = new NitrateSettings();

    armConfig.setPIDSettings(
        new PIDSettings()
            .setPID(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD)
            .setFeedforwardMode(PIDFeedforwardMode.kArm)
            .setGravitationalFeedforward(Constants.Arm.kG)
            .setMinwrapConfig(new MinwrapConfig.Disabled())
            .setMotionProfileAccelLimit(Constants.Arm.AccelerationLimit)
            .setMotionProfileDeaccelLimit(Constants.Arm.DeaccelerationLimit)
            .setMotionProfileVelocityLimit(Constants.Arm.VelocityLimit)
            .setISaturation(Constants.Arm.errorUnit)
            .setIZone(Constants.Arm.finalOutputDeg),
        PIDConfigSlot.kSlot0);

    armConfig.setPIDSettings(
        new PIDSettings()
            .setPID(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD)
            .setFeedforwardMode(PIDFeedforwardMode.kArm)
            .setGravitationalFeedforward(Constants.Arm.kG)
            .setMinwrapConfig(new MinwrapConfig.Disabled())
            .setMotionProfileAccelLimit(Constants.Arm.AccelerationLimit)
            .setMotionProfileDeaccelLimit(Constants.Arm.DeaccelerationLimit)
            .setMotionProfileVelocityLimit(Constants.Arm.slowVelocityLimit)
            .setISaturation(Constants.Arm.errorUnit)
            .setIZone(Constants.Arm.finalOutputDeg),
        PIDConfigSlot.kSlot1);

    armConfig.setFeedbackSensorSettings(
        new FeedbackSensorSettings()
            .setSensorToMechanismRatio(Constants.Arm.sensorToArm)
            .setFeedbackSensor(
                new FeedbackSensor.CanandmagRelative(
                    Constants.Arm.armEncoderId, Constants.Arm.motorShaftToSensorShaft)));

    armConfig.setOutputSettings(
        new OutputSettings()
            .setIdleMode(Constants.Arm.motorIdleMode)
            .setInvert(Constants.Arm.motorInvert));

    armConfig.setElectricalLimitSettings(
        new ElectricalLimitSettings()
            .setBusCurrentLimit(Constants.Arm.supplyCurrentLimitAmps)
            .setStatorCurrentLimit(Constants.Arm.statorCurrentLimitAmps));

    armConfig.setFramePeriodSettings(
        new FramePeriodSettings()
            .setEnabledPIDDebugFrames(
                new EnabledDebugFrames()
                    .setKgControlEffort(Constants.debugPIDModeEnabled)
                    .setKpControlEffort(Constants.debugPIDModeEnabled)
                    .setTotalControlEffort(Constants.debugPIDModeEnabled)));

    CanandmagSettings settings = new CanandmagSettings();
    CanandmagSettings EncoderConfigStatus = armEncoder.setSettings(settings, 0.02, 5);

    NitrateSettings motorConfigStatus = armMotor.setSettings(armConfig, 0.02, 5);

    if (!motorConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate " + armMotor.getAddress().getDeviceId() + " (Arm motor) failed to configure",
          false);
    }
    if (!EncoderConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Canandmag "
              + armEncoder.getAddress().getDeviceId()
              + " (Arm encoder) failed to configure",
          false);
    }
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.requestedPosDeg = lastRequestedPosDeg;
    inputs.PositionDegrees =
        Units.rotationsToDegrees(armMotor.getPosition()) - Constants.Arm.OffsetEncoderDeg;
    inputs.armConnected = armMotor.isConnected();
    inputs.voltage = armMotor.getBusVoltageFrame().getValue();
    inputs.velocityDegSec = Units.rotationsToDegrees(armMotor.getVelocity());
    inputs.SupplyCurrentAmps = armMotor.getBusCurrent();
    inputs.StatorCurrentAmps = armMotor.getStatorCurrent();
    inputs.TempCelsius = armMotor.getMotorTemperatureFrame().getData();
    inputs.armEncoderConnected = armEncoder.isConnected();
    inputs.voltage = armMotor.getAppliedVoltageFrame().getValue();
    inputs.encoderRotations = armMotor.getPosition();
    if (Constants.debugPIDModeEnabled) {
      inputs.kPeffort = armMotor.getPIDDebugFrames().kPControlEffortFrame.getValue();
      inputs.kGeffort = armMotor.getPIDDebugFrames().kGControlEffortFrame.getValue();
      inputs.totalEffort = armMotor.getPIDDebugFrames().totalControlEffortFrame.getValue();
    }
  }
  // You need method in ArmIO as well to do Override Remember to check - Personal Note / Reminder
  //

  @Override
  public void setHomePosition() {
    stopArmMotor(IdleMode.kBrake);
    armMotor.setPosition(Units.degreesToRotations(Constants.Arm.OffsetEncoderDeg));
  }

  @Override
  public void requestPosition(double requestSetpoint) {
    armMotor.setRequest(
        PIDPositionRequest.setPosition(
            Units.degreesToRotations(requestSetpoint + Constants.Arm.OffsetEncoderDeg)));
    lastRequestedPosDeg = requestSetpoint;
  }

  @Override
  public void requestSlowPosition(double requestSetpoint) {
    armMotor.setRequest(
        SlowPIDPositionRequest.setPosition(
            Units.degreesToRotations(requestSetpoint + Constants.Arm.OffsetEncoderDeg)));
    lastRequestedPosDeg = requestSetpoint;
  }

  @Override
  public void setVoltage(double voltage) {
    armMotor.setVoltage(voltage);
    lastRequestedPosDeg = -1;
  }

  @Override
  public void stopArmMotor(IdleMode idleMode) {
    armMotor.stop(idleMode);
    lastRequestedPosDeg = -1;
  }

  @Override
  public Nitrate getNitrate() {
    return armMotor;
  }
}
