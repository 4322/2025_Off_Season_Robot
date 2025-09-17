package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.OutputSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.FeedbackSensor;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MinwrapConfig;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
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

  private final PIDPositionRequest armPIDPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);
  private final PIDPositionRequest armSlowPIDPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot1, 0).useMotionProfile(true);

  public ArmIONitrate() {
    armMotor = new Nitrate(Constants.Arm.armMotorId, MotorType.kCu60);
    armEncoder = new Canandmag(Constants.Arm.armEncoderId);

    PIDSettings armPIDSettings = new PIDSettings();
    armPIDSettings.setPID(Constants.Arm.armkP, Constants.Arm.armkI, Constants.Arm.armkD);
    armPIDSettings.setGravitationalFeedforward(Constants.Arm.kg);
    armPIDSettings.setMinwrapConfig(new MinwrapConfig.Disabled());
    armPIDSettings.setMotionProfileAccelLimit(Constants.Arm.AccelerationLimit);
    armPIDSettings.setMotionProfileDeaccelLimit(Constants.Arm.DeaccelerationLimit);
    armPIDSettings.setMotionProfileVelocityLimit(Constants.Arm.VelocityLimit);

    PIDSettings armSlowPIDSettings = new PIDSettings();
    armSlowPIDSettings.setPID(Constants.Arm.armkP, Constants.Arm.armkI, Constants.Arm.armkD);
    armSlowPIDSettings.setGravitationalFeedforward(Constants.Arm.kg);
    armSlowPIDSettings.setMinwrapConfig(new MinwrapConfig.Disabled());
    armSlowPIDSettings.setMotionProfileAccelLimit(Constants.Arm.AccelerationLimit);
    armSlowPIDSettings.setMotionProfileDeaccelLimit(Constants.Arm.DeaccelerationLimit);
    armSlowPIDSettings.setMotionProfileVelocityLimit(Constants.Arm.slowVelocityLimit);

    NitrateSettings armConfig = new NitrateSettings();

    armConfig
        .getFeedbackSensorSettings()
        .setFeedbackSensor(
            new FeedbackSensor.CanandmagRelative(
                Constants.Arm.armEncoderId, Constants.Arm.motorShaftToSensorShaft))
        .setSensorToMechanismRatio(Constants.Arm.sensorToArm);

    armConfig
        .setPIDSettings(armPIDSettings, PIDConfigSlot.kSlot0)
        .getPIDSettings(PIDConfigSlot.kSlot0);
    armConfig
        .setPIDSettings(armSlowPIDSettings, PIDConfigSlot.kSlot1)
        .getPIDSettings(PIDConfigSlot.kSlot1);

    CanandmagSettings settings = new CanandmagSettings();
    CanandmagSettings armEncoderConfigStatus = armEncoder.setSettings(settings, 0.02, 5);

    NitrateSettings armConfigStatus = armMotor.setSettings(armConfig, 0.02, 5);
    OutputSettings armMotorOutputSettings = new OutputSettings();
    armMotorOutputSettings.setIdleMode(Constants.Arm.motorIdleMode);
    armMotorOutputSettings.setInvert(Constants.Arm.armMotorInvert);

    if (!armConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate " + armMotor.getAddress().getDeviceId() + " (Arm motor) failed to configure",
          false);
    }
    if (!armEncoderConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Canandmag "
              + armEncoder.getAddress().getDeviceId()
              + " (Arm encoder) failed to configure",
          false);
    }
  }

  @Override
  public void updateInputs(ArmIOInputs armInputs) {
    armInputs.requestedPosDeg = lastRequestedPosDeg;
    armInputs.armPositionDegrees =
        Units.rotationsToDegrees(armMotor.getPosition()) - Constants.Arm.armOffsetEncoderDeg;
    armInputs.armConnected = armMotor.isConnected();
    armInputs.voltage = armMotor.getBusVoltageFrame().getValue();
    armInputs.velocityDegSec = Units.rotationsToDegrees(armMotor.getVelocity());
    armInputs.armSupplyCurrentAmps = armMotor.getBusCurrent();
    armInputs.armStatorCurrentAmps = armMotor.getStatorCurrent();
    armInputs.armTempCelsius = armMotor.getMotorTemperatureFrame().getData();
    armInputs.armEncoderConnected = armEncoder.isConnected();
  }
  // You need method in ArmIO as well to do Override Remember to check - Personal Note / Reminder

  @Override
  public void setHomePosition() {
    stopArmMotor(IdleMode.kBrake);
    armMotor.setPosition(Units.degreesToRotations(Constants.Arm.armOffsetEncoderDeg));
  }

  @Override
  public void requestPosition(double requestSetpoint) {
    armMotor.setRequest(
        armPIDPositionRequest.setPosition(
            Units.degreesToRotations(requestSetpoint + Constants.Arm.armOffsetEncoderDeg)));
    lastRequestedPosDeg = requestSetpoint;
  }

  @Override
  public void requestSlowPosition(double requestSetpoint) {
    armMotor.setRequest(
        armSlowPIDPositionRequest.setPosition(
            Units.degreesToRotations(requestSetpoint + Constants.Arm.armOffsetEncoderDeg)));
    lastRequestedPosDeg = requestSetpoint;
  }

  @Override
  public void stopArmMotor(IdleMode idleMode) {
    armMotor.stop(idleMode);
    lastRequestedPosDeg = -1;
  }
}
