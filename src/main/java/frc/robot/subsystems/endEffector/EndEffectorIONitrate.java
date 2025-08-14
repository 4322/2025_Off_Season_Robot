package frc.robot.subsystems.endEffector;

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
import frc.robot.subsystems.endEffector.EndEffectorIO.EndEffectorIOInputs.gamePiece;

public class EndEffectorIONitrate implements EndEffectorIO {
  private Nitrate endEffectorMotor;
  private Canandcolor endEffectorSensor;

  private NitrateSettings endEffectorMotorConfig = new NitrateSettings();
  private CanandcolorSettings endEffectorSensorConfig = new CanandcolorSettings();

  private double previousRequestedVoltage = -999;

  public EndEffectorIONitrate() {
    endEffectorMotor = new Nitrate(Constants.EndEffector.endEffectorMotorId, MotorType.kCu60);
    endEffectorSensor = new Canandcolor(Constants.EndEffector.endEffectorSensorId);

    configMotor();
    NitrateSettings endEffectorMotorConfigStatus =
        endEffectorMotor.setSettings(endEffectorMotorConfig, 0.02, 5);
    if (!endEffectorMotorConfigStatus.allSettingsReceived()) {
      DriverStation.reportError(
          "Nitrate "
              + endEffectorMotor.getAddress().getDeviceId()
              + " error (End Effector Motor); Did not receive settings",
          null);
    }

    configSensor();
    CanandcolorSettings endEffectorSensorConfigStatus =
        endEffectorSensor.setSettings(endEffectorSensorConfig, 0.02, 5);
    if (!endEffectorSensorConfigStatus.allSettingsReceived()) {
      DriverStation.reportError(
          "Canandcolor "
              + endEffectorSensor.getAddress().getDeviceId()
              + " error (End Effector Sensor); Did not receive settings",
          null);
    }
  }

  private void configMotor() {
    // TODO add other settings for motor
    ElectricalLimitSettings endEffectorMotorElectricalLimitSettings = new ElectricalLimitSettings();
    endEffectorMotorElectricalLimitSettings.setBusCurrentLimit(
        Constants.EndEffector.motorBusCurrentLimit);
    endEffectorMotorElectricalLimitSettings.setBusCurrentLimitTime(
        Constants.EndEffector.motorBusCurrentLimitTime);
    endEffectorMotorElectricalLimitSettings.setStatorCurrentLimit(
        Constants.EndEffector.motorStatorCurrentLimit);
    endEffectorMotorConfig.setElectricalLimitSettings(endEffectorMotorElectricalLimitSettings);

    OutputSettings endEffectorMotorOutputSettings = new OutputSettings();
    endEffectorMotorOutputSettings.setIdleMode(Constants.EndEffector.motorIdleMode);
    endEffectorMotorOutputSettings.setInvert(Constants.EndEffector.motorInvert);
    endEffectorMotorConfig.setOutputSettings(endEffectorMotorOutputSettings);
  }

  private void configSensor() {
    // TODO add other settings for sensor
    //
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.endEffectorMotorConnected = endEffectorMotor.isConnected();
    inputs.endEffectorMotorSpeedRotationsPerSec = endEffectorMotor.getVelocity();
    inputs.endEffectorMotorStatorCurrentAmps = endEffectorMotor.getStatorCurrent();
    inputs.endEffectorMotorTempCelcius = endEffectorMotor.getMotorTemperatureFrame().getValue();
    inputs.endEffectorMotorBusCurrentAmps = endEffectorMotor.getBusCurrent();
    inputs.endEffectorMotorAppliedVolts = endEffectorMotor.getBusVoltageFrame().getValue();

    inputs.endEffectorSensorConnected = endEffectorSensor.isConnected();
    inputs.endEffectorSensorProximity = endEffectorSensor.getProximity();
    inputs.endEffectorSensorColorBlue = endEffectorSensor.getBlue();
    inputs.endEffectorSensorColorGreen = endEffectorSensor.getGreen();
    inputs.endEffectorSensorColorRed = endEffectorSensor.getRed();

    inputs.isCoralProximityDetected = endEffectorSensor.getProximity() < Constants.EndEffector.sensorCoralProximityThreshold;
    inputs.isAlgaeProximityDetected = endEffectorSensor.getProximity() < Constants.EndEffector.sensorAlgaeProximityThreshold
    && endEffectorSensor.getProximity() > Constants.EndEffector.sensorCoralProximityThreshold; // TODO Assuming algae is farther from sensor than coral is


      // Enable color detection based on Constant setting
      if (Constants.EndEffector.useSensorColor) {
        if (inputs.endEffectorSensorProximity < Constants.EndEffector.sensorAlgaeProximityThreshold) {
        
        // Green detected is within range; Blue detected is within range; Red detected is below
        // threshold
        if (inputs.endEffectorSensorColorGreen > Constants.EndEffector.sensorGreenDetectGreenLower
            && inputs.endEffectorSensorColorGreen < Constants.EndEffector.sensorGreenDetectGreenUpper
            && inputs.endEffectorSensorColorBlue > Constants.EndEffector.sensorGreenDetectBlueLower
            && inputs.endEffectorSensorColorBlue < Constants.EndEffector.sensorGreenDetectBlueUpper
            && inputs.endEffectorSensorColorRed < Constants.EndEffector.sensorGreenDetectRed) {
          inputs.sensorPieceDetected = gamePiece.ALGAE;

          // All colors detected are above threshold
        } else if (inputs.endEffectorSensorColorGreen > Constants.EndEffector.sensorWhiteDetectGreen
            && inputs.endEffectorSensorColorBlue > Constants.EndEffector.sensorWhiteDetectBlue
            && inputs.endEffectorSensorColorRed > Constants.EndEffector.sensorWhiteDetectRed) {
          inputs.sensorPieceDetected = gamePiece.CORAL;
        } else {
          inputs.sensorPieceDetected = gamePiece.UNKNOWN;
        }
      } else {
        inputs.sensorPieceDetected = gamePiece.NONE;
      }
    } else {
      if (inputs.endEffectorSensorProximity < Constants.EndEffector.sensorAlgaeProximityThreshold) {
        inputs.sensorPieceDetected = gamePiece.ALGAE;
      } else if (inputs.endEffectorSensorProximity < Constants.EndEffector.sensorCoralProximityThreshold) {
        inputs.sensorPieceDetected = gamePiece.CORAL;
      } else {
        inputs.sensorPieceDetected = gamePiece.NONE;
      }
    }
  }

  @Override
  public void setEndEffectorMotorVoltage(double voltage) {
    if (voltage != previousRequestedVoltage) {
      previousRequestedVoltage = voltage;
      endEffectorMotor.setVoltage(voltage);
    }
  }

  @Override
  // This covers both stopping motor as well as setting brake/coast mode
  public void stopEndEffectorMotor(IdleMode idleMode) {
    endEffectorMotor.stop(idleMode);
  }

}
