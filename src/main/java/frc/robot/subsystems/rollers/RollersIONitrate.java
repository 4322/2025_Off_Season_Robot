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

    initMotorConfig();
    NitrateSettings rollersMotorConfigStatus =
        rollersMotor.setSettings(rollersMotorConfig, 0.02, 5);
    if (!rollersMotorConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + rollersMotor.getAddress().getDeviceId()
              + " error (Rollers Motor); Did not receive settings",
          false);
    }
  }

  private void initMotorConfig() {
    rollersMotorConfig.getElectricalLimitSettings()
        .setBusCurrentLimit(Constants.Rollers.motorBusCurrentLimit)
        .setBusCurrentLimitTime(Constants.Rollers.motorBusCurrentLimitTime)
        .setStatorCurrentLimit(Constants.Rollers.motorStatorCurrentLimit);
    
    rollersMotorConfig.getOutputSettings()
        .setIdleMode(Constants.Rollers.motorIdleMode)
        .setInvert(Constants.Rollers.motorInvert);
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
  public void stopRollersMotor(IdleMode mode) {
    previousRequestedVoltage = -999;
    rollersMotor.stop(mode);
  }
}
