package frc.robot.subsystems.rollers;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.OutputSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class RollersIONitrate implements RollersIO {
  private Nitrate rollersMotor;

  private NitrateSettings rollersMotorConfig = new NitrateSettings();

  private double previousRequestedVoltage = -999;

  public RollersIONitrate() {
    rollersMotor = new Nitrate(Constants.Rollers.rollersMotorId, MotorType.kCu60);

    configMotor();
    NitrateSettings rollersMotorConfigStatus =
        rollersMotor.setSettings(rollersMotorConfig, 0.02, 5);
    if (!rollersMotorConfigStatus.allSettingsReceived()) {
      DriverStation.reportError(
          "Nitrate "
              + rollersMotor.getAddress().getDeviceId()
              + " error (Rollers Motor); Did not receive settings",
          null);
    }
  }

  private void configMotor() { // TODO rename this and all other ones
    // TODO add other settings for motor
    ElectricalLimitSettings rollersMotorElectricalLimitSettings = new ElectricalLimitSettings();
    rollersMotorElectricalLimitSettings.setBusCurrentLimit(Constants.Rollers.motorBusCurrentLimit);
    rollersMotorElectricalLimitSettings.setBusCurrentLimitTime(
        Constants.Rollers.motorBusCurrentLimitTime);
    rollersMotorElectricalLimitSettings.setStatorCurrentLimit(
        Constants.Rollers.motorStatorCurrentLimit);
    rollersMotorConfig.setElectricalLimitSettings(rollersMotorElectricalLimitSettings);

    OutputSettings rollersMotorOutputSettings = new OutputSettings();
    rollersMotorOutputSettings.setIdleMode(Constants.Rollers.motorIdleMode);
    rollersMotorOutputSettings.setInvert(Constants.Rollers.motorInvert);
    rollersMotorConfig.setOutputSettings(rollersMotorOutputSettings);
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.rollersMotorConnected = rollersMotor.isConnected();
    inputs.rollersMotorAppliedVoltage = rollersMotor.getBusVoltageFrame().getValue();
    inputs.rollersMotorBusCurrentAmps = rollersMotor.getBusCurrent();
    inputs.rollersMotorStatorCurrentAmps = rollersMotor.getStatorCurrent();
    inputs.rollersMotorTempCelcius = rollersMotor.getMotorTemperatureFrame().getValue();
    inputs.rollersMotorSpeedRotationsPerSec = rollersMotor.getVelocity();
  }

  @Override
  public void setRollersMotorVoltage(double voltage) {
    if (voltage != previousRequestedVoltage) {
      rollersMotor.setVoltage(voltage);
      previousRequestedVoltage = voltage;
    }
  }

  @Override
  public void stopRollersMotor(IdleMode mode) { // TODO clear previous requested voltage and and for all other subsystems.
    rollersMotor.stop(mode);
  }
}
