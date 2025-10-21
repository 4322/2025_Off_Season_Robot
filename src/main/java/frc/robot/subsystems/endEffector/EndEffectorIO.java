package frc.robot.subsystems.endEffector;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

  @AutoLog
  public static class EndEffectorIOInputs {
    public boolean motorConnected = false;
    public double statorCurrentAmps = 0.0;
    public double busCurrentAmps = 0.0;
    public double motorTempCelcius = 0.0;
    public double speedRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double controllerTempCelcius = 0.0;

    public boolean sensorConnected = false;
    public double sensorProximity = 0.0;
    public double sensorColorRed = 0.0;
    public double sensorColorGreen = 0.0;
    public double sensorColorBlue = 0.0;

    public boolean isRigatoniProximityDetected = false;
    public boolean isMeatballProximityDetected = false;

    public enum gamePiece {
      NONE,
      MEATBALL,
      RIGATONI,
      UNKNOWN
    }

    public gamePiece sensorPieceDetected = gamePiece.NONE;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void stopNitrate(IdleMode idleMode) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}

  // For sim mode
  public default void simRigatoniHeld() {}

  public default void simMeatballHeld() {}

  public default void simRigatoniReleased() {}

  public default void simMeatballReleased() {}

  // for tuning
  public default Nitrate getNitrate() {
    return null;
  }
}
