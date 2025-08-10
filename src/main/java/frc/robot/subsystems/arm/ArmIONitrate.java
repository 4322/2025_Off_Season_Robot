package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.AtomicBondMode;
import com.reduxrobotics.motorcontrol.nitrate.types.FeedbackSensor;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.InvertMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MinwrapConfig;
import com.reduxrobotics.motorcontrol.nitrate.types.MotionProfileMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import com.reduxrobotics.motorcontrol.requests.PIDPositionRequest;
import com.reduxrobotics.motorcontrol.requests.PIDVelocityRequest;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.util.SwerveUtil.SwerveModuleConstants;

public class ArmIONitrate implements ArmIO {

  private final Nitrate armMotor;
  private final Canandmag armEncoder;

  private final PIDVelocityRequest armPIDVelocityRequest =
      new PIDVelocityRequest(PIDConfigSlot.kSlot0, 0);
  private final PIDPositionRequest armPIDPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);

  public ArmIONitrate(SwerveModuleConstants constants, GyroIO gyro) {
    armMotor = new Nitrate(constants.turnMotorId, MotorType.kCu60);
    armEncoder = new Canandmag(constants.turnEncoderId);

    NitrateSettings armConfig = new NitrateSettings();
    armConfig
        .getAtomicBondSettings()
        .setAtomicBondMode(AtomicBondMode.kSwerveModule)
        .setAtomicSwerveConstants(
            armMotor,
            armEncoder,
            gyro.getGyro(),
            constants.driveMotorGearRatio,
            constants.turnCouplingRatio);
    armConfig
        .getOutputSettings()
        .setIdleMode(IdleMode.kBrake)
        .setInvert(constants.driveMotorInverted ? InvertMode.kInverted : InvertMode.kNotInverted);
    armConfig.setElectricalLimitSettings(constants.driveElectricalLimitSettings);
    armConfig.getFeedbackSensorSettings().setSensorToMechanismRatio(constants.driveMotorGearRatio);
    armConfig.setPIDSettings(constants.driveMotorGains, PIDConfigSlot.kSlot0);
    NitrateSettings driveConfigStatus = armMotor.setSettings(armConfig, 0.02, 5);

    NitrateSettings turnConfig = new NitrateSettings();
    turnConfig.getAtomicBondSettings().setAtomicBondMode(AtomicBondMode.kSwerveModule);
    turnConfig
        .getOutputSettings()
        .setIdleMode(IdleMode.kBrake)
        .setInvert(constants.turnMotorInverted ? InvertMode.kInverted : InvertMode.kNotInverted);
    turnConfig
        .getFeedbackSensorSettings()
        .setFeedbackSensor(
            new FeedbackSensor.CanandmagRelative(
                constants.turnEncoderId, constants.turnMotorGearRatio));
    turnConfig.setElectricalLimitSettings(constants.turnElectricalLimitSettings);
    turnConfig
        .setPIDSettings(constants.turnMotorGains, PIDConfigSlot.kSlot0)
        .getPIDSettings(PIDConfigSlot.kSlot0)
        .setMotionProfileMode(MotionProfileMode.kTrapezoidal)
        .setMinwrapConfig(new MinwrapConfig.Enabled());
    NitrateSettings armConfigStatus = armMotor.setSettings(turnConfig, 0.02, 5);

    CanandmagSettings settings = new CanandmagSettings();
    settings.setInvertDirection(constants.turnEncoderInverted);
    CanandmagSettings armEncoderConfigStatus = armEncoder.setSettings(settings, 0.02, 5);

    if (!armConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + armMotor.getAddress().getDeviceId()
              + " (Swerve turn motor) failed to configure",
          false);
    }
    if (!armEncoderConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Canandmag "
              + armEncoder.getAddress().getDeviceId()
              + " (Swerve turn encoder) failed to configure",
          false);
    }
  }

  @Override
  public void updateInputs(ArmIOInputs armInputs) {
    armInputs.armPositionRad = Units.rotationsToRadians(armMotor.getPosition());
    armInputs.armConnected = armMotor.isConnected();
    armInputs.armVelocityRadPerSec = Units.rotationsToRadians(armMotor.getVelocity());
    armInputs.armSupplyCurrentAmps = armMotor.getBusCurrent();
    armInputs.armStatorCurrentAmps = armMotor.getStatorCurrent();
    armInputs.armTempCelsius = armMotor.getMotorTemperatureFrame().getData();

    armInputs.armEncoderConnected = armEncoder.isConnected();
    armInputs.armAbsolutePosition = Rotation2d.fromRotations(armEncoder.getAbsPosition());
    armInputs.armPosition = Rotation2d.fromRotations(armEncoder.getPosition());
  }

  public void setArmVelocity(double armWheelVelocityRadPerSec) {
    armMotor.setRequest(
        armPIDVelocityRequest.setPosition(
            Units.radiansToRotations(
                armWheelVelocityRadPerSec))); // TODO: Wait for API to get fixed
  }

  public void setArmPosition(Rotation2d armWheelPosition) {
    armMotor.setRequest(armPIDPositionRequest.setPosition(armWheelPosition.getRotations()));
  }
}
