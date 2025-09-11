package frc.robot.subsystems.indexer;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
    public boolean indexerMotorLeftConnected = false;
    public double indexerMotorLeftAppliedVoltage = 0.0;
    public double indexerMotorLeftBusCurrentAmps = 0.0;
    public double indexerMotorLeftStatorCurrentAmps = 0.0;
    public double indexerMotorLeftTempCelcius = 0.0;
    public double indexerMotorLeftSpeedRotationsPerSec = 0.0;

    public boolean indexerMotorRightConnected = false;
    public double indexerMotorRightAppliedVoltage = 0.0;
    public double indexerMotorRightBusCurrentAmps = 0.0;
    public double indexerMotorRightStatorCurrentAmps = 0.0;
    public double indexerMotorRightTempCelcius = 0.0;
    public double indexerMotorRightSpeedRotationsPerSec = 0.0;

    public boolean indexerSensorConnected = false;
    public boolean indexerSensorTriggered = false;
    public double indexerSensorProximity = 0.0;

    public boolean pickupAreaSensorConnected = false;
    public boolean pickupAreaSensorTriggered = false;
    public double pickupAreaSensorProximity = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setIndexerMotorsVoltage(double voltage) {}

  public default void stopIndexerMotor(IdleMode mode) {}
}
