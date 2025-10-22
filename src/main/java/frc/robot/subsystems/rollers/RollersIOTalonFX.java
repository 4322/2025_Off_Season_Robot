package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class RollersIOTalonFX implements RollersIO {
  private TalonFX rollersMotor;

  private double previousRequestedVoltage = -999;

  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  public RollersIOTalonFX() {
    rollersMotor = new TalonFX(Constants.Rollers.motorId);

    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.Rollers.statorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Rollers.busCurrentLimit;

    motorConfigs.MotorOutput.Inverted = Constants.Rollers.motorInvertPhoenix;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode feederConfigStatus = rollersMotor.getConfigurator().apply(motorConfigs);

    if (feederConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + rollersMotor.getDeviceID()
              + " error (End Effector): "
              + feederConfigStatus.getDescription(),
          false);
    }
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {

    inputs.connected = rollersMotor.isConnected();
    inputs.appliedVoltage = rollersMotor.getMotorVoltage().getValueAsDouble();
    inputs.busCurrentAmps = rollersMotor.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrentAmps = rollersMotor.getStatorCurrent().getValueAsDouble();
    inputs.motorTempCelcius = rollersMotor.getDeviceTemp().getValueAsDouble();
    inputs.speedRotationsPerSec = rollersMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    if (voltage != previousRequestedVoltage) {
      previousRequestedVoltage = voltage;
      rollersMotor.setVoltage(voltage);
    }
  }

  @Override
  public void stop() {
    rollersMotor.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    rollersMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public TalonFX getTalonFX() {
    return rollersMotor;
  }
}
