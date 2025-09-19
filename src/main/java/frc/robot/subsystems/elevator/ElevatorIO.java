package frc.robot.subsystems.elevator;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {

    public double requestedPosMeters;

    public double leaderheightMeters = 0.0;
    public double leaderVoltage = 0.0;
    public boolean leaderConnected = false;
    public double leaderVelocityMetersPerSecond = 0.0;
    public double leaderSupplyAmps = 0.0;
    public double leaderStatorAmps = 0.0;
    public double leadertempCelcius = 0.0;

    public double followerHeightMeters = 0.0;
    public double followerVoltage = 0.0;
    public boolean followerConnected = false;
    public double followerVelocityMetersPerSecond = 0.0;
    public double followerSupplyAmps = 0.0;
    public double followerStatorAmps = 0.0;
    public double followertempCelcius = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void requestSlowHeightMeters(double heightMeters) {}

  public default void requestHeightMeters(double heightMeters) {}

  public default void setVoltage(double voltage) {}

  public default void setPosition(double elevatorPositionMeters) {}

  public default void stop(IdleMode idleMode) {}

  public default Nitrate getNitrate() {
    return null;
  } // for tuning
}
