package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double heightMeters = 0.0;
    //current setpoint
    public double velocityMetersSecond = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmpsF = 0.0; // {leader, follower}
    public double statorCurrentAmpsF = 0.0; // {leader, follower}
    public double tempCelciusF = 0.0; // {leader, follower}
    public double supplyCurrentAmpsL = 0.0; // {leader, follower}
    public double statorCurrentAmpsL = 0.0; // {leader, follower}
    public double tempCelciusL = 0.0; // {leader, follower}
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void requestHeight(double heightMeters) {}

  public default void setVoltage(double voltage) {}

  public default void setElevatorEncoder() {}

  public default void stopElevator(IdleMode mode) {}
}
