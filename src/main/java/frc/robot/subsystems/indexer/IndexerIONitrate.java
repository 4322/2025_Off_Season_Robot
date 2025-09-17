package frc.robot.subsystems.indexer;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.OutputSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class IndexerIONitrate implements IndexerIO {
  private Nitrate indexerMotorRight;
  private Nitrate indexerMotorLeft;
  private Canandcolor indexerSensor;
  private Canandcolor pickupAreaSensor;

  private NitrateSettings indexerMotorRightConfig = new NitrateSettings();
  private NitrateSettings indexerMotorLeftConfig = new NitrateSettings();
  private CanandcolorSettings indexerSensorConfig = new CanandcolorSettings();
  private CanandcolorSettings pickupAreaSensorConfig = new CanandcolorSettings();

  private double previousRequestedVoltage = -999;

  public IndexerIONitrate() {
    indexerMotorRight = new Nitrate(Constants.Indexer.indexerMotorRightId, MotorType.kCu60);
    indexerMotorLeft = new Nitrate(Constants.Indexer.indexerMotorLeftId, MotorType.kCu60);
    indexerSensor = new Canandcolor(Constants.Indexer.indexerSensorId);
    pickupAreaSensor = new Canandcolor(Constants.Indexer.pickupAreaSensorId);

    initMotorConfig();
    NitrateSettings indexerMotorConfigStatus =
        indexerMotorRight.setSettings(indexerMotorRightConfig, 0.02, 5);
    if (!indexerMotorConfigStatus.allSettingsReceived()) {
      DriverStation.reportError(
          "Nitrate "
              + indexerMotorRight.getAddress().getDeviceId()
              + " error (Indexer Motor); Did not receive settings",
          false);
    }
    NitrateSettings indexerMotorLeftConfigStatus =
        indexerMotorLeft.setSettings(indexerMotorLeftConfig, 0.02, 5);
    if (!indexerMotorLeftConfigStatus.allSettingsReceived()) {
      DriverStation.reportError(
          "Nitrate "
              + indexerMotorLeft.getAddress().getDeviceId()
              + " error (Indexer Motor Left); Did not receive settings",
          false);
    }

    configSensor();
    CanandcolorSettings indexerSensorConfigStatus =
        indexerSensor.setSettings(indexerSensorConfig, 0.02, 5);
    if (!indexerSensorConfigStatus.allSettingsReceived()) {
      DriverStation.reportError(
          "Canandcolor "
              + indexerSensor.getAddress().getDeviceId()
              + " error (Indexer Sensor); Did not receive settings",
          false);
    }

    CanandcolorSettings pickupAreaSensorConfigStatus =
        pickupAreaSensor.setSettings(pickupAreaSensorConfig, 0.02, 5);
    if (!pickupAreaSensorConfigStatus.allSettingsReceived()) {
      DriverStation.reportError(
          "Canandcolor "
              + pickupAreaSensor.getAddress().getDeviceId()
              + " error (Pickup Area Sensor); Did not receive settings",
          false);
    }
  }

  private void initMotorConfig() {
    // These are shared between both motors for now
    ElectricalLimitSettings indexerMotorElectricalLimitSettings = new ElectricalLimitSettings();
    indexerMotorElectricalLimitSettings.setBusCurrentLimit(Constants.Indexer.motorBusCurrentLimit);
    indexerMotorElectricalLimitSettings.setBusCurrentLimitTime(
        Constants.Indexer.motorBusCurrentLimitTime);
    indexerMotorElectricalLimitSettings.setStatorCurrentLimit(
        Constants.Indexer.motorStatorCurrentLimit);
    indexerMotorRightConfig.setElectricalLimitSettings(indexerMotorElectricalLimitSettings);
    indexerMotorLeftConfig.setElectricalLimitSettings(indexerMotorElectricalLimitSettings);

    OutputSettings indexerMotorRightOutputSettings = new OutputSettings();
    indexerMotorRightOutputSettings.setIdleMode(Constants.Indexer.motorIdleMode);
    indexerMotorRightOutputSettings.setInvert(Constants.Indexer.motorRightInvert);
    indexerMotorRightConfig.setOutputSettings(indexerMotorRightOutputSettings);

    OutputSettings indexerMotorLeftOutputSettings = new OutputSettings();
    indexerMotorLeftOutputSettings.setIdleMode(Constants.Indexer.motorIdleMode);
    indexerMotorLeftOutputSettings.setInvert(Constants.Indexer.motorLeftInvert);
    indexerMotorLeftConfig.setOutputSettings(indexerMotorLeftOutputSettings);
  }

  private void configSensor() {
    // TODO add other settings for sensors
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {

    inputs.indexerMotorLeftConnected = indexerMotorLeft.isConnected();
    inputs.indexerMotorLeftAppliedVoltage = indexerMotorLeft.getBusVoltageFrame().getValue();
    inputs.indexerMotorLeftBusCurrentAmps = indexerMotorLeft.getBusCurrent();
    inputs.indexerMotorLeftStatorCurrentAmps = indexerMotorLeft.getStatorCurrent();
    inputs.indexerMotorLeftTempCelcius = indexerMotorLeft.getMotorTemperatureFrame().getValue();
    inputs.indexerMotorLeftSpeedRotationsPerSec = indexerMotorLeft.getVelocity();

    inputs.indexerMotorRightConnected = indexerMotorRight.isConnected();
    inputs.indexerMotorRightAppliedVoltage = indexerMotorRight.getBusVoltageFrame().getValue();
    inputs.indexerMotorRightBusCurrentAmps = indexerMotorRight.getBusCurrent();
    inputs.indexerMotorRightStatorCurrentAmps = indexerMotorRight.getStatorCurrent();
    inputs.indexerMotorRightTempCelcius = indexerMotorRight.getMotorTemperatureFrame().getValue();
    inputs.indexerMotorRightSpeedRotationsPerSec = indexerMotorRight.getVelocity();

    inputs.indexerSensorConnected = indexerSensor.isConnected();
    inputs.indexerSensorTriggered =
        indexerSensor.getProximity() < Constants.Indexer.indexerSensorMax;
    inputs.indexerSensorProximity = indexerSensor.getProximity();

    inputs.pickupAreaSensorConnected = pickupAreaSensor.isConnected();
    inputs.pickupAreaSensorTriggered =
        pickupAreaSensor.getProximity() < Constants.Indexer.pickupAreaSensorMax;
    inputs.pickupAreaSensorProximity = pickupAreaSensor.getProximity();
  }

  @Override
  public void setIndexerMotorsVoltage(double voltage) {
    if (voltage != previousRequestedVoltage) {
      indexerMotorRight.setVoltage(voltage);
      indexerMotorLeft.setVoltage(voltage);
      previousRequestedVoltage = voltage;
    }
  }

  @Override
  public void stopIndexerMotor(IdleMode mode) {
    previousRequestedVoltage = -999;
    indexerMotorRight.stop(mode);
    indexerMotorLeft.stop(mode);
  }
}
