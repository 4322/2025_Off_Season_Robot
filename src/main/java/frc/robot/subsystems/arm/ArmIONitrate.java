package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.FeedbackSensor;
import com.reduxrobotics.motorcontrol.nitrate.types.MinwrapConfig;
import com.reduxrobotics.motorcontrol.nitrate.types.MotionProfileMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import com.reduxrobotics.motorcontrol.requests.PIDPositionRequest;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIONitrate implements ArmIO {

  private final Nitrate armMotor;

  private final PIDPositionRequest armPIDPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);

  public ArmIONitrate() {
    armMotor = new Nitrate(Constants.Arm.armMotorId, MotorType.kCu60);

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

  }

  @Override
  public void updateInputs(ArmIOInputs armInputs) {
    armInputs.armPositionDegrees = Units.rotationsToDegrees(armMotor.getPosition());
    armInputs.armConnected = armMotor.isConnected();
    armInputs.armSupplyCurrentAmps = armMotor.getBusCurrent();
    armInputs.armStatorCurrentAmps = armMotor.getStatorCurrent();
    armInputs.armTempCelsius = armMotor.getMotorTemperatureFrame().getData();
  }

  @Override
  public void requestPosition(Rotation2d angle) {
    armMotor.setRequest(
        armPIDPositionRequest.setPosition(
            angle.getRotations() - angle.minus(Rotation2d.fromDegrees(90)).getRotations()));
  }
}
