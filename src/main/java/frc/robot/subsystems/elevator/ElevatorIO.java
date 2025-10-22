package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {

    public double requestedPosMeters;
    public double requestedPosRotations;

    public double leaderheightMeters = 0.0;
    public double leaderVoltage = 0.0;
    public boolean leaderConnected = false;
    public double velMetersPerSecond = 0.0;
    public double leaderSupplyAmps = 0.0;
    public double leaderStatorAmps = 0.0;
    public double leadertempCelcius = 0.0;
    public double leaderEncoderRotations = 0.0;
    public double kGeffort;
    public double kPeffort;
    public double kIeffort;
    public double totalEffort;
    public double feedbackError;
    public double leaderControllerTempCelcius = 0.0;
    public double followerControllerTempCelcius = 0.0;

    public double followerHeightMeters = 0.0;
    public double followerVoltage = 0.0;
    public boolean followerConnected = false;
    public double followerVelocityMetersPerSecond = 0.0;
    public double followerSupplyAmps = 0.0;
    public double followerStatorAmps = 0.0;
    public double followertempCelcius = 0.0;
    public double followerEncoderRotations = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void requestSlowHeightMeters(double heightMeters) {}

  public default void requestHeightMeters(double heightMeters) {}

  public default void setVoltage(double voltage) {}

  public default void setPosition(double elevatorPositionMeters) {}

  // public default void stop(IdleMode idleMode) {}

  public default void stop() {}

  // public default Nitrate getNitrate() {
  //   return null;
  // }

  public default void enableBrakeMode(boolean enable) {}

  public default TalonFX getTalonFX() {
    return null;
  } // for tuning
}
