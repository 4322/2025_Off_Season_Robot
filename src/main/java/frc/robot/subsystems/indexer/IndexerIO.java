package frc.robot.subsystems.indexer;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
    public boolean leftConnected = false;
    public double leftAppliedVoltage = 0.0;
    public double leftBusCurrentAmps = 0.0;
    public double leftStatorCurrentAmps = 0.0;
    public double leftTempCelcius = 0.0;
    public double leftSpeedRotationsPerSec = 0.0;

    public boolean rightConnected = false;
    public double rightAppliedVoltage = 0.0;
    public double rightBusCurrentAmps = 0.0;
    public double rightStatorCurrentAmps = 0.0;
    public double rightTempCelcius = 0.0;
    public double rightSpeedRotationsPerSec = 0.0;

    public boolean indexerSensorConnected = false;
    public boolean indexerSensorTriggered = false;
    public double indexerSensorProximity = 0.0;

    public boolean pickupAreaSensorConnected = false;
    public boolean pickupAreaSensorTriggered = false;
    public double pickupAreaSensorProximity = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void stop(IdleMode mode) {}
}
