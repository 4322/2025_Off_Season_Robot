package frc.robot.subsystems.endEffector;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

  @AutoLog
  public static class EndEffectorIOInputs {
    public boolean endEffectorMotorConnected = false;
    public double endEffectorMotorStatorCurrentAmps = 0.0;
    public double endEffectorMotorBusCurrentAmps = 0.0;
    public double endEffectorMotorTempCelcius = 0.0;
    public double endEffectorMotorSpeedRotationsPerSec = 0.0;
    public double endEffectorMotorAppliedVolts = 0.0;

    public boolean endEffectorSensorConnected = false;
    public double endEffectorSensorProximity = 0.0;
    public double endEffectorSensorColorRed = 0.0;
    public double endEffectorSensorColorGreen = 0.0;
    public double endEffectorSensorColorBlue = 0.0;

    public enum gamePiece {
      NONE,
      ALGAE,
      CORAL,
      UNKNOWN
    }

    public gamePiece sensorPieceDetected = gamePiece.NONE;

    public boolean currentDetectionPickupTriggered = false;
    public boolean currentDetectionReleaseTriggered = false;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setEndEffectorMotorVoltage(double voltage) {}

  public default void stopEndEffectorMotor(IdleMode idleMode) {}

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
