package frc.robot.subsystems.endEffector;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

  @AutoLog
  public static class EndEffectorIOInputs {
    public double endEffectorMotorStatorCurrentAmps = 0.0;
    public double endEffectorMotorBusCurrentAmps = 0.0;
    public double endEffectorMotorTempCelcius = 0.0;
    public double endEffectorMotorSpeedRotationsPerSec = 0.0;

    public double endEffectorSensorProximity = 0.0;

    public enum colorDetected {
      NONE,
      GREEN,
      WHITE
    }

    public colorDetected sensorColorDetected = colorDetected.NONE;

    public boolean currentDetectionPickupTriggered = false;
    public boolean currentDetectionReleaseTriggered = false;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setEndEffectorMotorVoltage(double voltage) {}

  public default void stopEndEffectorMotor(IdleMode idleMode) {}

  public default void enableBrakeMode(boolean enable) {}

  public default boolean isCurrentDetectionPickupTriggered() {
    return false;
  }

  public default boolean isCurrentDetectionReleaseTriggered() {
    return false;
  }

  public default boolean isCoralProximityDetected() {
    return false;
  }

  public default boolean isAlgaeProximityDetected() {
    return false;
  }
}
