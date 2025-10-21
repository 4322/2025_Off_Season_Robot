package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class IndexerIOTalonFX implements IndexerIO {
  private TalonFX indexerMotorLeft;
  private TalonFX indexerMotorRight;
  private Canandcolor indexerSensor;
  private Canandcolor pickupAreaSensor;

  private double previousRequestedVoltage = -999;

  private TalonFXConfiguration motorConfigsLeft = new TalonFXConfiguration();
  private TalonFXConfiguration motorConfigsRight = new TalonFXConfiguration();
  private CanandcolorSettings indexerSensorConfig = new CanandcolorSettings();
  private CanandcolorSettings pickupAreaSensorConfig = new CanandcolorSettings();

  public IndexerIOTalonFX() {
    indexerMotorLeft = new TalonFX(Constants.Indexer.leftId);
    indexerMotorRight = new TalonFX(Constants.Indexer.rightId);
    indexerSensor = new Canandcolor(Constants.Indexer.indexerSensorId);
    pickupAreaSensor = new Canandcolor(Constants.Indexer.pickupAreaSensorId);

    motorConfigsLeft.CurrentLimits.StatorCurrentLimit = Constants.EndEffector.statorCurrentLimit;
    motorConfigsLeft.CurrentLimits.SupplyCurrentLimit = Constants.EndEffector.busCurrentLimit;

    motorConfigsLeft.MotorOutput.Inverted = Constants.Indexer.leftMotorInvertPhoenix;
    motorConfigsLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfigsLeft.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigsLeft.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode feederConfigStatusLeft = indexerMotorLeft.getConfigurator().apply(motorConfigsLeft);

    motorConfigsRight.CurrentLimits.StatorCurrentLimit = Constants.EndEffector.statorCurrentLimit;
    motorConfigsRight.CurrentLimits.SupplyCurrentLimit = Constants.EndEffector.busCurrentLimit;

    motorConfigsRight.MotorOutput.Inverted = Constants.Indexer.rightMotorInvertPhoenix;
    motorConfigsRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfigsRight.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigsRight.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode feederConfigStatusRight = indexerMotorLeft.getConfigurator().apply(motorConfigsLeft);

    if (feederConfigStatusLeft != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + indexerMotorLeft.getDeviceID()
              + " error (End Effector): "
              + feederConfigStatusLeft.getDescription(),
          false);
    }

    if (feederConfigStatusRight != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + indexerMotorRight.getDeviceID()
              + " error (End Effector): "
              + feederConfigStatusRight.getDescription(),
          false);
    }

    configSensor();
    CanandcolorSettings indexerSensorConfigStatus =
        indexerSensor.setSettings(indexerSensorConfig, 0.1, 5);
    if (!indexerSensorConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Canandcolor "
              + indexerSensor.getAddress().getDeviceId()
              + " error (Indexer Sensor); Did not receive settings",
          false);
    }

    CanandcolorSettings pickupAreaSensorConfigStatus =
        pickupAreaSensor.setSettings(pickupAreaSensorConfig, 0.1, 5);
    if (!pickupAreaSensorConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Canandcolor "
              + pickupAreaSensor.getAddress().getDeviceId()
              + " error (Pickup Area Sensor); Did not receive settings",
          false);
    }
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {

    inputs.leftConnected = indexerMotorLeft.isConnected();
    inputs.leftAppliedVoltage = indexerMotorLeft.getMotorVoltage().getValueAsDouble();
    inputs.leftBusCurrentAmps = indexerMotorLeft.getSupplyCurrent().getValueAsDouble();
    inputs.leftStatorCurrentAmps = indexerMotorLeft.getStatorCurrent().getValueAsDouble();
    inputs.leftTempCelcius = indexerMotorLeft.getDeviceTemp().getValueAsDouble();
    inputs.leftSpeedRotationsPerSec = indexerMotorLeft.getVelocity().getValueAsDouble();

    inputs.rightConnected = indexerMotorRight.isConnected();
    inputs.rightAppliedVoltage = indexerMotorRight.getMotorVoltage().getValueAsDouble();
    inputs.rightBusCurrentAmps = indexerMotorRight.getSupplyCurrent().getValueAsDouble();
    inputs.rightStatorCurrentAmps = indexerMotorRight.getStatorCurrent().getValueAsDouble();
    inputs.rightTempCelcius = indexerMotorRight.getDeviceTemp().getValueAsDouble();
    inputs.rightSpeedRotationsPerSec = indexerMotorRight.getVelocity().getValueAsDouble();

    inputs.indexerSensorConnected = indexerSensor.isConnected();
    inputs.indexerSensorTriggered =
        indexerSensor.getProximity() < Constants.Indexer.indexerSensorMax;
    inputs.indexerSensorProximity = indexerSensor.getProximity();

    inputs.pickupAreaSensorConnected = pickupAreaSensor.isConnected();
    inputs.pickupAreaSensorTriggered =
        pickupAreaSensor.getProximity() < Constants.Indexer.pickupAreaSensorMax;
    inputs.pickupAreaSensorProximity = pickupAreaSensor.getProximity();
  }

  private void configSensor() {
    indexerSensorConfig.setColorFramePeriod(0); // reduce CAN bus traffic
    pickupAreaSensorConfig.setColorFramePeriod(0);
  }

  @Override
  public void setVoltage(double voltage) {
    if (voltage != previousRequestedVoltage || Constants.continuousNitrateRequestsEnabled) {
      previousRequestedVoltage = voltage;
      indexerMotorRight.setVoltage(voltage);
      indexerMotorLeft.setVoltage(voltage);
    }
  }

  public void stop() {
    indexerMotorRight.stopMotor();
    indexerMotorLeft.stopMotor();
  }

  public void enableBrakeMode(boolean enable) {
    indexerMotorRight.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    indexerMotorLeft.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
