package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double heightMeters = 0.0;
    //current setpoint
    public boolean elevatorConnected = false;
    public boolean elevatorEncoderConnected = false;
    public double velocityMetersSecond = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0; // {leader, follower}
    public double statorCurrentAmps = 0.0; // {leader, follower}
    public double tempCelcius = 0.0; // {leader, follower}
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void requestHeight(double heightMeters) {}

  public default void setVoltage(double voltage) {}

  public default void setElevatorEncoder() {}

  public default void setNeutralMode(IdleMode mode) {}
}
