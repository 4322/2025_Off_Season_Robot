package frc.robot.subsystems.rollers;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.FeedbackSensorSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.OutputSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class RollersIONitrate implements RollersIO {
  private Nitrate rollersMotor;

  private NitrateSettings motorConfig = new NitrateSettings();

  private double prevRequestedVoltage = -999;

  public RollersIONitrate() {
    rollersMotor = new Nitrate(Constants.Rollers.motorId, MotorType.kCu60);

    initMotorConfig();
    NitrateSettings motorConfigStatus = rollersMotor.setSettings(motorConfig, 0.02, 5);
    if (!motorConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + rollersMotor.getAddress().getDeviceId()
              + " error (Rollers Motor); Did not receive settings",
          false);
    }
  }

  private void initMotorConfig() {
    motorConfig.setElectricalLimitSettings(
        new ElectricalLimitSettings()
            .setBusCurrentLimit(Constants.Rollers.busCurrentLimit)
            .setBusCurrentLimitTime(Constants.Rollers.busCurrentLimitTime)
            .setStatorCurrentLimit(Constants.Rollers.statorCurrentLimit));

    motorConfig.setOutputSettings(
        new OutputSettings()
            .setIdleMode(Constants.Rollers.idleMode)
            .setInvert(Constants.Rollers.invert));

    motorConfig.setFeedbackSensorSettings(FeedbackSensorSettings.defaultSettings());
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.connected = rollersMotor.isConnected();
    inputs.appliedVoltage = rollersMotor.getBusVoltageFrame().getValue();
    inputs.busCurrentAmps = rollersMotor.getBusCurrent();
    inputs.statorCurrentAmps = rollersMotor.getStatorCurrent();
    inputs.tempCelcius = rollersMotor.getMotorTemperatureFrame().getValue();
    inputs.speedRotationsPerSec = rollersMotor.getVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    if (voltage != prevRequestedVoltage) {
      rollersMotor.setVoltage(voltage);
      prevRequestedVoltage = voltage;
    }
  }

  @Override
  public void stop(IdleMode mode) {
    prevRequestedVoltage = -999;
    rollersMotor.stop(mode);
  }
}
