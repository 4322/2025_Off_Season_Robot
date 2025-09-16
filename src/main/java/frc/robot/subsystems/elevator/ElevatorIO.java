package frc.robot.subsystems.elevator;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {

    public double requestedPosMeters;

    public double leaderMotorheightMeters = 0.0;
    public double leaderMotorVoltage = 0.0;
    public boolean leaderElevatorMotorConnected = false;
    public double leaderMotorVelocityMetersPerSecond = 0.0;
    public double leaderMotorAppliedVoltage = 0.0;
    public double leaderMotorSupplyCurrentAmps = 0.0;
    public double leaderMotorStatorCurrentAmps = 0.0;
    public double leaderMotortempCelcius = 0.0;

    public double followerMotorheightMeters = 0.0;
    public double followerMotorVoltage = 0.0;
    public boolean followerElevatorMotorConnected = false;
    public double followerMotorVelocityMetersPerSecond = 0.0;
    public double followerMotorAppliedVoltage = 0.0;
    public double followerMotorSupplyCurrentAmps = 0.0;
    public double followerMotorStatorCurrentAmps = 0.0;
    public double followerMotortempCelcius = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void requestSlowHeightMeters(double heightMeters) {}

  public default void requestHeightMeters(double heightMeters) {}

  public default void setVoltage(double voltage) {}

  public default void setPosition(double elevatorPositionMeters) {}

  public default void stop(IdleMode idleMode) {}
}
