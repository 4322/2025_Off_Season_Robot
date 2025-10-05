package frc.robot.subsystems.drive;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.FeedbackSensorSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.FramePeriodSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.OutputSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.FeedbackSensor;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.InvertMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import com.reduxrobotics.motorcontrol.requests.PIDPositionRequest;
import com.reduxrobotics.motorcontrol.requests.PIDVelocityRequest;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.ClockUtil;
import frc.robot.util.SwerveUtil.SwerveModuleConstants;

public class ModuleIONitrate implements ModuleIO {
  private final Nitrate driveMotor;
  private final Nitrate turnMotor;
  private final Canandmag turnEncoder;
  private final SwerveModuleConstants constants;

  private final PIDVelocityRequest drivePIDOpenLoopRequest =
      new PIDVelocityRequest(PIDConfigSlot.kSlot1, 0);
  private final PIDVelocityRequest drivePIDVelocityRequest =
      new PIDVelocityRequest(PIDConfigSlot.kSlot0, 0);
  private final PIDPositionRequest turnPIDPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);

  public ModuleIONitrate(SwerveModuleConstants constants, GyroIOBoron gyro) {
    driveMotor = new Nitrate(constants.driveMotorId, MotorType.kCu60);
    turnMotor = new Nitrate(constants.turnMotorId, MotorType.kCu60);
    turnEncoder = new Canandmag(constants.turnEncoderId);
    this.constants = constants;

    NitrateSettings driveConfig = new NitrateSettings();

    driveConfig.setOutputSettings(
        OutputSettings.defaultSettings()
            .setIdleMode(IdleMode.kBrake)
            .setInvert(
                constants.driveMotorInverted ? InvertMode.kInverted : InvertMode.kNotInverted));
    driveConfig.setElectricalLimitSettings(constants.driveElectricalLimitSettings);
    driveConfig.setFeedbackSensorSettings(
        FeedbackSensorSettings.defaultSettings()
            .setSensorToMechanismRatio(constants.driveMotorGearRatio));
    driveConfig.setPIDSettings(constants.driveMotorGains, PIDConfigSlot.kSlot0);
    driveConfig.setFramePeriodSettings(FramePeriodSettings.defaultSettings());
    NitrateSettings driveConfigStatus = driveMotor.setSettings(driveConfig, 0.02, 5);

    NitrateSettings turnConfig = new NitrateSettings();
    turnConfig.setOutputSettings(
        OutputSettings.defaultSettings()
            .setIdleMode(IdleMode.kBrake)
            .setInvert(
                constants.turnMotorInverted ? InvertMode.kInverted : InvertMode.kNotInverted));
    turnConfig.setFeedbackSensorSettings(
        FeedbackSensorSettings.defaultSettings()
            .setFeedbackSensor(
                new FeedbackSensor.CanandmagAbsolute(
                    constants.turnEncoderId, constants.turnMotorGearRatio)));
    turnConfig.setElectricalLimitSettings(constants.turnElectricalLimitSettings);
    turnConfig.setPIDSettings(constants.turnMotorGains, PIDConfigSlot.kSlot0);
    turnConfig.setFramePeriodSettings(FramePeriodSettings.defaultSettings());
    NitrateSettings turnConfigStatus = turnMotor.setSettings(turnConfig, 0.02, 5);

    CanandmagSettings settings = new CanandmagSettings();
    settings.setInvertDirection(constants.turnEncoderInverted);
    CanandmagSettings turnEncoderConfigStatus = turnEncoder.setSettings(settings, 0.02, 5);

    if (!driveConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + driveMotor.getAddress().getDeviceId()
              + " (Swerve drive motor) failed to configure",
          false);
    }
    if (!turnConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + turnMotor.getAddress().getDeviceId()
              + " (Swerve turn motor) failed to configure",
          false);
    }
    if (!turnEncoderConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Canandmag "
              + turnEncoder.getAddress().getDeviceId()
              + " (Swerve turn encoder) failed to configure",
          false);
    }

    PIDSettings slot1Settings =
        new PIDSettings()
            .setStaticFeedforward(constants.driveMotorGains.getStaticFeedforward().get())
            .setVelocityFeedforward(constants.driveMotorGains.getVelocityFeedforward().get())
            .setPID(0, 0, 0)
            .setGravitationalFeedforward(0)
            .setMotionProfileVelocityLimit(0)
            .setMotionProfileAccelLimit(0)
            .setMotionProfileDeaccelLimit(0);

    PIDSettings driveSlot1ConfigStatus =
        driveMotor.setPIDSettings(slot1Settings, PIDConfigSlot.kSlot1);

    if (!driveSlot1ConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + driveMotor.getAddress().getDeviceId()
              + " (Swerve drive motor) PID Slot 1 failed to configure",
          false);
    }
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = driveMotor.isConnected();
    inputs.drivePositionMeters =
        driveMotor.getPosition() * (2 * Math.PI) * constants.driveWheelRadius;
    inputs.driveVelocityMetersPerSec =
        driveMotor.getVelocity() * (2 * Math.PI) * constants.driveWheelRadius;
    inputs.driveAppliedVolts = driveMotor.getBusVoltageFrame().getData();
    inputs.driveSupplyCurrentAmps = driveMotor.getBusCurrent();
    inputs.driveStatorCurrentAmps = driveMotor.getStatorCurrent();
    inputs.driveTempCelsius = driveMotor.getMotorTemperatureFrame().getData();
    inputs.driveControllerTempCelsius = driveMotor.getControllerTemperatureFrame().getData();

    inputs.turnConnected = turnMotor.isConnected();
    inputs.turnPosition = Rotation2d.fromRotations(turnMotor.getPosition() - 0.5);
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnMotor.getVelocity());
    inputs.turnAppliedVolts = driveMotor.getBusVoltageFrame().getData();
    inputs.turnSupplyCurrentAmps = turnMotor.getBusCurrent();
    inputs.turnStatorCurrentAmps = turnMotor.getStatorCurrent();
    inputs.turnTempCelsius = turnMotor.getMotorTemperatureFrame().getData();
    inputs.turnControllerTempCelsius = turnMotor.getControllerTemperatureFrame().getData();

    inputs.turnEncoderConnected = turnEncoder.isConnected();
    inputs.turnEncoderAbsPosition = turnEncoder.getAbsPosition();
    inputs.turnEncoderPosition = turnEncoder.getPosition();
  }

  @Override
  public void setDriveOpenLoop(double driveWheelVelocityRadPerSec) {
    driveMotor.setRequest(
        drivePIDOpenLoopRequest.setVelocity(Units.radiansToRotations(driveWheelVelocityRadPerSec)));
  }

  @Override
  public void setDriveVelocity(double driveWheelVelocityRadPerSec) {
    driveMotor.setRequest(
        drivePIDVelocityRequest.setVelocity(Units.radiansToRotations(driveWheelVelocityRadPerSec)));
  }

  @Override
  public void setTurnPosition(Rotation2d turnWheelPosition) {
    // Convert back to motor rotations and apply input modulus to handle double variable precision
    // edge case
    turnMotor.setRequest(
        turnPIDPositionRequest.setPosition(
            ClockUtil.inputModulus(turnWheelPosition.getRotations() + 0.5, 0, 1)));
  }

  @Override
  public Nitrate getTurnNitrate() {
    return turnMotor;
  }

  @Override
  public Nitrate getDriveNitrate() {
    return driveMotor;
  }
}
