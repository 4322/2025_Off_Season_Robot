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

  private NitrateSettings motorRightConfig = new NitrateSettings();
  private NitrateSettings motorLeftConfig = new NitrateSettings();
  private CanandcolorSettings indexerSensorConfig = new CanandcolorSettings();
  private CanandcolorSettings pickupAreaSensorConfig = new CanandcolorSettings();

  private double prevRequestedVoltage = -999;

  public IndexerIONitrate() {
    indexerMotorRight = new Nitrate(Constants.Indexer.rightId, MotorType.kCu60);
    indexerMotorLeft = new Nitrate(Constants.Indexer.leftId, MotorType.kCu60);
    indexerSensor = new Canandcolor(Constants.Indexer.indexerSensorId);
    pickupAreaSensor = new Canandcolor(Constants.Indexer.pickupAreaSensorId);

    initMotorConfig();
    NitrateSettings motorRightConfigStatus =
        indexerMotorRight.setSettings(motorRightConfig, 0.02, 5);
    if (!motorRightConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + indexerMotorRight.getAddress().getDeviceId()
              + " error (Indexer Motor); Did not receive settings",
          false);
    }
    NitrateSettings motorLeftConfigStatus = indexerMotorLeft.setSettings(motorLeftConfig, 0.02, 5);
    if (!motorLeftConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + indexerMotorLeft.getAddress().getDeviceId()
              + " error (Indexer Motor Left); Did not receive settings",
          false);
    }

    configSensor();
    CanandcolorSettings indexerSensorConfigStatus =
        indexerSensor.setSettings(indexerSensorConfig, 0.02, 5);
    if (!indexerSensorConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Canandcolor "
              + indexerSensor.getAddress().getDeviceId()
              + " error (Indexer Sensor); Did not receive settings",
          false);
    }

    CanandcolorSettings pickupAreaSensorConfigStatus =
        pickupAreaSensor.setSettings(pickupAreaSensorConfig, 0.02, 5);
    if (!pickupAreaSensorConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Canandcolor "
              + pickupAreaSensor.getAddress().getDeviceId()
              + " error (Pickup Area Sensor); Did not receive settings",
          false);
    }
  }

  private void initMotorConfig() {
    motorLeftConfig.setElectricalLimitSettings(
        new ElectricalLimitSettings()
            .setBusCurrentLimit(Constants.Indexer.busCurrentLimit)
            .setBusCurrentLimitTime(Constants.Indexer.busCurrentLimitTime)
            .setStatorCurrentLimit(Constants.Indexer.statorCurrentLimit));

    motorRightConfig.setElectricalLimitSettings(
        new ElectricalLimitSettings()
            .setBusCurrentLimit(Constants.Indexer.busCurrentLimit)
            .setBusCurrentLimitTime(Constants.Indexer.busCurrentLimitTime)
            .setStatorCurrentLimit(Constants.Indexer.statorCurrentLimit));

    motorLeftConfig.setOutputSettings(
        new OutputSettings()
            .setIdleMode(Constants.Indexer.idleMode)
            .setInvert(Constants.Indexer.leftInvert));

    motorRightConfig.setOutputSettings(
        new OutputSettings()
            .setIdleMode(Constants.Indexer.idleMode)
            .setInvert(Constants.Indexer.rightInvert));
  }

  private void configSensor() {}

  @Override
  public void updateInputs(IndexerIOInputs inputs) {

    inputs.leftConnected = indexerMotorLeft.isConnected();
    inputs.leftAppliedVoltage = indexerMotorLeft.getBusVoltageFrame().getValue();
    inputs.leftBusCurrentAmps = indexerMotorLeft.getBusCurrent();
    inputs.leftStatorCurrentAmps = indexerMotorLeft.getStatorCurrent();
    inputs.leftTempCelcius = indexerMotorLeft.getMotorTemperatureFrame().getValue();
    inputs.leftSpeedRotationsPerSec = indexerMotorLeft.getVelocity();

    inputs.rightConnected = indexerMotorRight.isConnected();
    inputs.rightAppliedVoltage = indexerMotorRight.getBusVoltageFrame().getValue();
    inputs.rightBusCurrentAmps = indexerMotorRight.getBusCurrent();
    inputs.rightStatorCurrentAmps = indexerMotorRight.getStatorCurrent();
    inputs.rightTempCelcius = indexerMotorRight.getMotorTemperatureFrame().getValue();
    inputs.rightSpeedRotationsPerSec = indexerMotorRight.getVelocity();

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
  public void setVoltage(double voltage) {
    if (voltage != prevRequestedVoltage) {
      indexerMotorRight.setVoltage(voltage);
      indexerMotorLeft.setVoltage(voltage);
      prevRequestedVoltage = voltage;
    }
  }

  @Override
  public void stop(IdleMode mode) {
    prevRequestedVoltage = -999;
    indexerMotorRight.stop(mode);
    indexerMotorLeft.stop(mode);
  }
}
