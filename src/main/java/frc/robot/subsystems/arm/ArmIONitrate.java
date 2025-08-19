package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.FeedbackSensor;
import com.reduxrobotics.motorcontrol.nitrate.types.MinwrapConfig;
import com.reduxrobotics.motorcontrol.nitrate.types.MotionProfileMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import com.reduxrobotics.motorcontrol.requests.PIDPositionRequest;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIONitrate implements ArmIO {

  private final Nitrate armMotor;
  private final Canandmag armEncoder;

  private final PIDPositionRequest armPIDPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);

  public ArmIONitrate() {
    armMotor = new Nitrate(Constants.Arm.armMotorId, MotorType.kCu60);
    armEncoder = new Canandmag(Constants.Arm.armEncoderId);

    NitrateSettings armConfig = new NitrateSettings();
    armConfig
        .getFeedbackSensorSettings()
        .setFeedbackSensor(
            new FeedbackSensor.CanandmagRelative(
                Constants.Arm.armEncoderId, Constants.Arm.armMotorGearRatio));
    armConfig
        .setPIDSettings(Constants.Arm.armMotorGains, PIDConfigSlot.kSlot0)
        .getPIDSettings(PIDConfigSlot.kSlot0)
        .setMotionProfileMode(MotionProfileMode.kTrapezoidal)
        .setMinwrapConfig(new MinwrapConfig.Enabled());

    CanandmagSettings settings = new CanandmagSettings();
    CanandmagSettings armEncoderConfigStatus = armEncoder.setSettings(settings, 0.02, 5);

    NitrateSettings armConfigStatus = armMotor.setSettings(armConfig, 0.02, 5);

    if (!armConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + armMotor.getAddress().getDeviceId()
              + " (Arm motor) failed to configure",
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
    armInputs.armPositionDegrees = Units.rotationsToDegrees(armMotor.getPosition());
    armInputs.armConnected = armMotor.isConnected();
    armInputs.armSupplyCurrentAmps = armMotor.getBusCurrent();
    armInputs.armStatorCurrentAmps = armMotor.getStatorCurrent();
    armInputs.armTempCelsius = armMotor.getMotorTemperatureFrame().getData();
    armInputs.armEncoderConnected = armEncoder.isConnected();
  }
  // You need method in ArmIO as well to do Override Remember to check - Personal Note / Reminder

  @Override
  public void setManualInitialization() {
    armMotor.setPosition(0);
  }

  @Override
  public void requestPosition(double requestSetpoint) {
    armMotor.setRequest(
        armPIDPositionRequest.setPosition(
            Constants.Arm.armOffsetEncoderDeg + Units.degreesToRotations(requestSetpoint)));
  }
}
