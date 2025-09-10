package frc.robot.subsystems.elevator;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {

    public double leaderMotorheightMeters = 0.0;
    // current setpoint
    public boolean leaderElevatorMotorConnected = false;
    public double leaderMotorVelocityMetersSecond = 0.0;
    public double leaderMotorAppliedVoltage = 0.0;
    public double leaderMotorSupplyCurrentAmps = 0.0; // {leader, follower}
    public double leaderMotorStatorCurrentAmps = 0.0; // {leader, follower}
    public double leaderMotortempCelcius = 0.0; // {leader, follower}
    // current setpoint
    public boolean followerElevatorMotorConnected = false;
    public double followerMotorAppliedVoltage = 0.0;
    public double followerMotorSupplyCurrentAmps = 0.0; // {leader, follower}
    public double followerMotorStatorCurrentAmps = 0.0; // {leader, follower}
    public double followerMotortempCelcius = 0.0; // {leader, follower}
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void requestSlowHeightMeters(double heightMeters) {}

  public default void requestHeightMeters(double heightMeters) {}

  public default void setVoltage(double voltage) {}

  public default void setPosition(double elevatorPositionMeters) {}

  public default void setNeutralMode(IdleMode idleMode) {}
}
