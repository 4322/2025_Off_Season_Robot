package frc.robot.subsystems.endEffector;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
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

  public EndEffectorIONitrate() {
    endEffectorMotor = new Nitrate(Constants.EndEffector.endEffectorMotorID, MotorType.kCu60);
    endEffectorSensor = new Canandcolor(Constants.EndEffector.endEffectorSensorID);

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
        Constants.EndEffector.MOTOR_BUS_CURRENT_LIMIT);
    endEffectorMotorElectricalLimitSettings.setBusCurrentLimitTime(
        Constants.EndEffector.MOTOR_BUS_CURRENT_LIMIT_TIME);
    endEffectorMotorElectricalLimitSettings.setStatorCurrentLimit(
        Constants.EndEffector.MOTOR_STATOR_CURRENT_LIMIT);
    endEffectorMotorConfig.setElectricalLimitSettings(endEffectorMotorElectricalLimitSettings);

    // Set output settings brake mode
    // Set invert flag w/ constants
    // Feedback sensor setting for position control
  }

  private void configSensor() {
    // TODO add other settings for sensor
    // 
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    // Voltage 
    // Whatever is in swerve
    inputs.endEffectorMotorSpeedRotationsPerSec = endEffectorMotor.getVelocity();
    inputs.endEffectorMotorStatorCurrentAmps = endEffectorMotor.getStatorCurrent();
    inputs.endEffectorMotorTempCelcius = endEffectorMotor.getMotorTemperatureFrame().getData();
    inputs.endEffectorMotorBusCurrentAmps = endEffectorMotor.getBusCurrent();

    inputs.endEffectorSensorProximity = endEffectorSensor.getProximity();
    inputs.endEffectorSensorColorBlue = endEffectorSensor.getBlue();
    inputs.endEffectorSensorColorGreen = endEffectorSensor.getGreen();
    inputs.endEffectorSensorColorRed = endEffectorSensor.getRed();

    if (endEffectorSensor.getProximity() < Constants.EndEffector.SENSOR_ALGAE_PROXIMITY_THRESHOLD) {
      if (endEffectorSensor.getGreen() > Constants.EndEffector.SENSOR_GREEN_DETECT_GREEN_LOWER
          && endEffectorSensor.getGreen() < Constants.EndEffector.SENSOR_GREEN_DETECT_GREEN_UPPER
          && endEffectorSensor.getBlue() > Constants.EndEffector.SENSOR_GREEN_DETECT_BLUE_LOWER
          && endEffectorSensor.getBlue() < Constants.EndEffector.SENSOR_GREEN_DETECT_BLUE_UPPER
          && endEffectorSensor.getRed() < Constants.EndEffector.SENSOR_GREEN_DETECT_RED) {
        inputs.sensorPieceDetected = gamePiece.ALGAE;
      } else if (endEffectorSensor.getGreen() > Constants.EndEffector.SENSOR_WHITE_DETECT_GREEN
          && endEffectorSensor.getBlue() > Constants.EndEffector.SENSOR_WHITE_DETECT_BLUE
          && endEffectorSensor.getRed() > Constants.EndEffector.SENSOR_WHITE_DETECT_RED) {
        inputs.sensorPieceDetected = gamePiece.CORAL;
      } else {
        inputs.sensorPieceDetected = gamePiece.UNKNOWN;
      }
    } else {
      inputs.sensorPieceDetected = gamePiece.NONE;
    }
    inputs.currentDetectionPickupTriggered = isCurrentDetectionPickupTriggered();
  }

  @Override
  public void setEndEffectorMotorVoltage(double voltage) {
    endEffectorMotor.setVoltage(voltage);
  }

  @Override
  // This covers both stopping motor as well as setting brake/coast mode
  public void stopEndEffectorMotor(IdleMode idleMode) {
    endEffectorMotor.stop(idleMode);
  }

  @Override
  // Returns whether a difference in current is detected after picking up piece (high -> low
  // current)
  public boolean isCurrentDetectionPickupTriggered() {
    return false; // TODO figure out how current detection works
  }

  @Override
  // Returns whether a difference in current is detected after releasing a piece (low -> high
  // current)
  public boolean isCurrentDetectionReleaseTriggered() {
    return false; // TODO figure out how current detection works
  }

  @Override
  public boolean isCoralProximityDetected() {
    return endEffectorSensor.getProximity() < Constants.EndEffector.SENSOR_CORAL_PROXIMITY_THRESHOLD;
  }

  @Override
  public boolean isAlgaeProximityDetected() {
    // This is assuming the coral will be closer to the sensor than the algae is when held
    return endEffectorSensor.getProximity() < Constants.EndEffector.SENSOR_ALGAE_PROXIMITY_THRESHOLD
        && endEffectorSensor.getProximity()
            > Constants.EndEffector.SENSOR_CORAL_PROXIMITY_THRESHOLD;
  }
}
