package frc.robot.subsystems.endEffector;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.FeedbackSensorSettings;
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

  private NitrateSettings motorConfig = new NitrateSettings();
  private CanandcolorSettings sensorConfig = new CanandcolorSettings();

  private double previousRequestedVoltage = -999;

  public EndEffectorIONitrate() {
    endEffectorMotor = new Nitrate(Constants.EndEffector.motorId, MotorType.kCu60);
    endEffectorSensor = new Canandcolor(Constants.EndEffector.sensorId);

    initMotorConfig();
    NitrateSettings endEffectorMotorConfigStatus =
        endEffectorMotor.setSettings(motorConfig, 0.02, 5);
    if (!endEffectorMotorConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + endEffectorMotor.getAddress().getDeviceId()
              + " error (End Effector Motor); Did not receive settings",
          false);
    }

    initSensorConfig();
    CanandcolorSettings endEffectorSensorConfigStatus =
        endEffectorSensor.setSettings(sensorConfig, 0.02, 5);
    if (!endEffectorSensorConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Canandcolor "
              + endEffectorSensor.getAddress().getDeviceId()
              + " error (End Effector Sensor); Did not receive settings",
          false);
    }
  }

  private void initMotorConfig() {
    motorConfig.setElectricalLimitSettings(
        new ElectricalLimitSettings()
            .setBusCurrentLimit(Constants.EndEffector.busCurrentLimit)
            .setBusCurrentLimitTime(Constants.EndEffector.busCurrentLimitTime)
            .setStatorCurrentLimit(Constants.EndEffector.statorCurrentLimit));

    motorConfig.setOutputSettings(
        new OutputSettings()
            .setIdleMode(Constants.EndEffector.motorIdleMode)
            .setInvert(Constants.EndEffector.motorInvert));

    motorConfig.setFeedbackSensorSettings(FeedbackSensorSettings.defaultSettings());
  }

  private void initSensorConfig() {}

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.motorConnected = endEffectorMotor.isConnected();
    inputs.speedRotationsPerSec = endEffectorMotor.getVelocity();
    inputs.statorCurrentAmps = endEffectorMotor.getStatorCurrent();
    inputs.tempCelcius = endEffectorMotor.getMotorTemperatureFrame().getValue();
    inputs.busCurrentAmps = endEffectorMotor.getBusCurrent();
    inputs.appliedVolts = endEffectorMotor.getBusVoltageFrame().getValue();

    inputs.sensorConnected = endEffectorSensor.isConnected();
    inputs.sensorProximity = endEffectorSensor.getProximity();
    inputs.sensorColorBlue = endEffectorSensor.getBlue();
    inputs.sensorColorGreen = endEffectorSensor.getGreen();
    inputs.sensorColorRed = endEffectorSensor.getRed();

    inputs.isCoralProximityDetected =
        endEffectorSensor.getProximity() < Constants.EndEffector.coralProximityThreshold;
    inputs.isAlgaeProximityDetected =
        endEffectorSensor.getProximity() < Constants.EndEffector.algaeProximityThreshold
            && !inputs.isCoralProximityDetected; // TODO Assuming algae is farther from sensor
    // than coral is

    // Enable color detection based on Constant setting
    if (Constants.EndEffector.useSensorColor) {
      if (inputs.isAlgaeProximityDetected) {

        // Green detected is within range; Blue detected is within range; Red detected is below
        // threshold
        if (inputs.sensorColorGreen > Constants.EndEffector.greenDetectGreenLower
            && inputs.sensorColorGreen < Constants.EndEffector.greenDetectGreenUpper
            && inputs.sensorColorBlue > Constants.EndEffector.greenDetectBlueLower
            && inputs.sensorColorBlue < Constants.EndEffector.greenDetectBlueUpper
            && inputs.sensorColorRed < Constants.EndEffector.greenDetectRed) {
          inputs.sensorPieceDetected = gamePiece.ALGAE;

          // All colors detected are above threshold
        } else if (inputs.sensorColorGreen > Constants.EndEffector.whiteDetectGreen
            && inputs.sensorColorBlue > Constants.EndEffector.whiteDetectBlue
            && inputs.sensorColorRed > Constants.EndEffector.whiteDetectRed) {
          inputs.sensorPieceDetected = gamePiece.CORAL;
        } else {
          inputs.sensorPieceDetected = gamePiece.UNKNOWN;
        }
      } else {
        inputs.sensorPieceDetected = gamePiece.NONE;
      }
    } else {
      if (inputs.isAlgaeProximityDetected) {
        inputs.sensorPieceDetected = gamePiece.ALGAE;
      } else if (inputs.isCoralProximityDetected) {
        inputs.sensorPieceDetected = gamePiece.CORAL;
      } else {
        inputs.sensorPieceDetected = gamePiece.NONE;
      }
    }
  }

  @Override
  public void setVoltage(double voltage) {
    if (voltage != previousRequestedVoltage) {
      previousRequestedVoltage = voltage;
      endEffectorMotor.setVoltage(voltage);
    }
  }

  @Override
  // This covers both stopping motor as well as setting brake/coast mode
  public void stop(IdleMode idleMode) {
    previousRequestedVoltage = -999;
    endEffectorMotor.stop(idleMode);
  }
}
