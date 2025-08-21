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

  public default void setEndEffectorMotorVoltage(double voltage) {}

  public default void stopEndEffectorMotor(IdleMode idleMode) {}
}
