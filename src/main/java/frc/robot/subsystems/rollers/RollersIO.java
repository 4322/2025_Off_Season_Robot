package frc.robot.subsystems.rollers;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    public boolean rollersMotorConnected = false;
    public double rollersMotorAppliedVoltage = 0.0;
    public double rollersMotorBusCurrentAmps = 0.0;
    public double rollersMotorStatorCurrentAmps = 0.0;
    public double rollersMotorTempCelcius = 0.0;
    public double rollersMotorSpeedRotationsPerSec = 0.0;
  }

  public default void updateInputs(RollersIOInputs inputs) {}

  public default void setRollersMotorVoltage(double voltage) {}

  public default void stopRollersMotor(IdleMode mode) {}
}
