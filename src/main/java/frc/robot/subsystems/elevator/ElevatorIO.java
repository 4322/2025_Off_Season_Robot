package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double heightMeters = 0.0;
    //current setpoint
    
    public double appliedVoltage = 0.0;
    public double[] supplyCurrentAmps = new double[] {}; // {leader, follower}
    public double[] statorCurrentAmps = new double[] {}; // {leader, follower}
    public double[] tempCelcius = new double[] {}; // {leader, follower}
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void requestHeight(double heightMeters) {}

  public default void setVoltage(double voltage) {}

  public default void seedPosition(double motorPositionRot) {}

  public default void stopElevator(IdleMode mode) {}

  public default void enableBrakeMode(boolean enable) {}
}
