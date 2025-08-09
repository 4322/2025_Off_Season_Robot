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
import frc.robot.subsystems.endEffector.EndEffectorIO.EndEffectorIOInputs.colorDetected;

public class EndEffectorIONitrate implements EndEffectorIO {
    private Nitrate endEffectorMotor;
    private Canandcolor endEffectorSensor;

    private NitrateSettings endEffectorMotorConfig = new NitrateSettings();
    private CanandcolorSettings endEffectorSensorConfig = new CanandcolorSettings();

    public EndEffectorIONitrate() {
        endEffectorMotor = new Nitrate(Constants.EndEffector.endEffectorMotorID, MotorType.kGenericBrushless); // TODO guess at motor type?
        endEffectorSensor = new Canandcolor(Constants.EndEffector.endEffectorSensorID);

        configMotor();
        NitrateSettings endEffectorMotorConfigStatus = endEffectorMotor.setSettings(endEffectorMotorConfig, 100, 1);
        if (!endEffectorMotorConfigStatus.allSettingsReceived()) {
            DriverStation.reportError("Nitrate " + endEffectorMotor.getAddress() + " error (End Effector Motor); Did not receive settings", null);
        }

        configSensor();
        CanandcolorSettings endEffectorSensorConfigStatus = endEffectorSensor.setSettings(endEffectorSensorConfig, 100, 1);
        if (!endEffectorSensorConfigStatus.allSettingsReceived()) {
            DriverStation.reportError("Canandcolor " + endEffectorSensor.getAddress() + " error (End Effector Sensor); Did not receive settings", null);
        }

    }

    private void configMotor() {
        // TODO add other settings for motor
        ElectricalLimitSettings endEffectorMotorElectricalLimitSettings = new ElectricalLimitSettings();
        endEffectorMotorElectricalLimitSettings.setBusCurrentLimit(Constants.EndEffector.MOTOR_BUS_CURRENT_LIMIT);
        endEffectorMotorElectricalLimitSettings.setBusCurrentLimitTime(Constants.EndEffector.MOTOR_BUS_CURRENT_LIMIT_TIME);
        endEffectorMotorElectricalLimitSettings.setStatorCurrentLimit(Constants.EndEffector.MOTOR_STATOR_CURRENT_LIMIT);
        endEffectorMotorConfig.setElectricalLimitSettings(endEffectorMotorElectricalLimitSettings);
    }

    private void configSensor() {
        // TODO add other settings for sensor

    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.endEffectorMotorSpeedRotationsPerSec = endEffectorMotor.getVelocity();
        inputs.endEffectorMotorStatorCurrentAmps = endEffectorMotor.getStatorCurrent();
        inputs.endEffectorMotorTempCelcius = endEffectorMotor.getMotorTemperatureFrame().getData();
        inputs.endEffectorMotorBusCurrentAmps = endEffectorMotor.getBusCurrent();

        inputs.endEffectorSensorProximity = endEffectorSensor.getProximity();
        if (endEffectorSensor.getGreen() > Constants.EndEffector.SENSOR_GREEN_THRESHOLD
        && endEffectorSensor.getBlue() < Constants.EndEffector.SENSOR_BLUE_THRESHOLD
        && endEffectorSensor.getRed() < Constants.EndEffector.SENSOR_RED_THRESHOLD) {
            inputs.sensorColorDetected = colorDetected.GREEN;
        } else if (endEffectorSensor.getGreen() < Constants.EndEffector.SENSOR_GREEN_THRESHOLD
        && endEffectorSensor.getBlue() < Constants.EndEffector.SENSOR_BLUE_THRESHOLD
        && endEffectorSensor.getRed() < Constants.EndEffector.SENSOR_RED_THRESHOLD) {
            inputs.sensorColorDetected = colorDetected.WHITE;
        } else {
            inputs.sensorColorDetected = colorDetected.NONE;
        }
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

}
