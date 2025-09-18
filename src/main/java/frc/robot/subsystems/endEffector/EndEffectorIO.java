package frc.robot.subsystems.endEffector;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

  @AutoLog
  public static class EndEffectorIOInputs {
    public boolean motorConnected = false;
    public double statorCurrentAmps = 0.0;
    public double busCurrentAmps = 0.0;
    public double tempCelcius = 0.0;
    public double speedRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;

    public boolean sensorConnected = false;
    public double sensorProximity = 0.0;
    public double sensorColorRed = 0.0;
    public double sensorColorGreen = 0.0;
    public double sensorColorBlue = 0.0;

    public boolean isCoralProximityDetected = false;
    public boolean isAlgaeProximityDetected = false;

    public enum gamePiece {
      NONE,
      ALGAE,
      CORAL,
      UNKNOWN
    }

    public gamePiece sensorPieceDetected = gamePiece.NONE;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void stop(IdleMode idleMode) {}
}
