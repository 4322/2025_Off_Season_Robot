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
  private Nitrate indexerMotor;
  private Canandcolor indexerSensor;
  private Canandcolor pickupAreaSensor;

  private NitrateSettings indexerMotorConfig = new NitrateSettings();
  private CanandcolorSettings indexerSensorConfig = new CanandcolorSettings();
  private CanandcolorSettings pickupAreaSensorConfig = new CanandcolorSettings();

  private double previousRequestedVoltage = -999;

  public IndexerIONitrate() {
    indexerMotor = new Nitrate(Constants.Indexer.indexerMotorId, MotorType.kCu60);
    indexerSensor = new Canandcolor(Constants.Indexer.indexerSensorId);
    pickupAreaSensor = new Canandcolor(Constants.Indexer.pickupAreaSensorId);

    initMotorConfig();
    NitrateSettings indexerMotorConfigStatus =
        indexerMotor.setSettings(indexerMotorConfig, 0.02, 5);
    if (!indexerMotorConfigStatus.allSettingsReceived()) {
      DriverStation.reportError(
          "Nitrate "
              + indexerMotor.getAddress().getDeviceId()
              + " error (Indexer Motor); Did not receive settings",
          null);
    }

    configSensor();
    CanandcolorSettings indexerSensorConfigStatus =
        indexerSensor.setSettings(indexerSensorConfig, 0.02, 5);
    if (!indexerSensorConfigStatus.allSettingsReceived()) {
      DriverStation.reportError(
          "Canandcolor "
              + indexerSensor.getAddress().getDeviceId()
              + " error (Indexer Sensor); Did not receive settings",
          null);
    }

    CanandcolorSettings pickupAreaSensorConfigStatus =
        pickupAreaSensor.setSettings(pickupAreaSensorConfig, 0.02, 5);
    if (!pickupAreaSensorConfigStatus.allSettingsReceived()) {
      DriverStation.reportError(
          "Canandcolor "
              + pickupAreaSensor.getAddress().getDeviceId()
              + " error (Pickup Area Sensor); Did not receive settings",
          null);
    }
  }

  private void initMotorConfig() {
    // TODO add other settings for motor
    ElectricalLimitSettings indexerMotorElectricalLimitSettings = new ElectricalLimitSettings();
    indexerMotorElectricalLimitSettings.setBusCurrentLimit(Constants.Indexer.motorBusCurrentLimit);
    indexerMotorElectricalLimitSettings.setBusCurrentLimitTime(
        Constants.Indexer.motorBusCurrentLimitTime);
    indexerMotorElectricalLimitSettings.setStatorCurrentLimit(
        Constants.Indexer.motorStatorCurrentLimit);
    indexerMotorConfig.setElectricalLimitSettings(indexerMotorElectricalLimitSettings);

    OutputSettings indexerMotorOutputSettings = new OutputSettings();
    indexerMotorOutputSettings.setIdleMode(Constants.Indexer.motorIdleMode);
    indexerMotorOutputSettings.setInvert(Constants.Indexer.motorInvert);
    indexerMotorConfig.setOutputSettings(indexerMotorOutputSettings);
  }

  private void configSensor() {
    // TODO add other settings for sensors
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerMotorConnected = indexerMotor.isConnected();
    inputs.indexerMotorAppliedVoltage = indexerMotor.getBusVoltageFrame().getValue();
    inputs.indexerMotorBusCurrentAmps = indexerMotor.getBusCurrent();
    inputs.indexerMotorStatorCurrentAmps = indexerMotor.getStatorCurrent();
    inputs.indexerMotorTempCelcius = indexerMotor.getMotorTemperatureFrame().getValue();
    inputs.indexerMotorSpeedRotationsPerSec = indexerMotor.getVelocity();

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
  public void setIndexerMotorVoltage(double voltage) {
    if (voltage != previousRequestedVoltage) {
      indexerMotor.setVoltage(voltage);
      previousRequestedVoltage = voltage;
    }
  }

  @Override
  public void stopIndexerMotor(IdleMode mode) {
    previousRequestedVoltage = -999;
    indexerMotor.stop(mode);
  }
}
