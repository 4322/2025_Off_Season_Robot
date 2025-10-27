package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    public boolean connected = false;
    public double appliedVoltage = 0.0;
    public double busCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double motorTempCelcius = 0.0;
    public double speedRotationsPerSec = 0.0;
    public double controllerTempCelcius = 0.0;
  }

  public default void updateInputs(RollersIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void stopNitrate(IdleMode mode) {}

  public default void stop() {}

  // for tuning
  public default TalonFX getTalonFX() {
    return null;
  }
}
