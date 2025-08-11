package frc.robot.subsystems.indexer;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
    public boolean indexerMotorConnected = false;
    public double indexerMotorAppliedVoltage = 0.0;
    public double indexerMotorBusCurrentAmps = 0.0;
    public double indexerMotorStatorCurrentAmps = 0.0;
    public double indexerMotorTempCelcius = 0.0;
    public double indexerMotorSpeedRotationsPerSec = 0.0;

    public boolean indexerSensorConnected = false;
    public boolean indexerSensorTriggered = false;
    public double indexerSensorProximity = 0.0;

    public boolean pickupAreaSensorConnected = false;
    public boolean pickupAreaSensorTriggered = false;
    public double pickupAreaSensorProximity = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setIndexerMotorVoltage(double voltage) {}

  public default void stopIndexerMotor(IdleMode mode) {}

  public default boolean isIndexerSensorTriggered() {
    return false;
  }

  public default boolean isPickupAreaSensorTriggered() {
    return false;
  }
}
