package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double requestedPosDeg;
    public double voltage = 0.0;
    public double velocityDegSec = 0.0;
    public double encoderArmRotations = 0.0;
    public boolean armConnected = false;
    public boolean armEncoderConnected = false;
    public double SupplyCurrentAmps = 0.0;
    public double StatorCurrentAmps = 0.0;
    public double motorTempCelsius = 0.0;
    public double[] tempCelcius = new double[] {};
    public double controllerTempCelsius = 0.0;
    public double PositionDegrees =
        0.0; // 0 is vertical to front of robot. Posititve clockwise looking from the left

    // ASk if we still need the PID debugging frames
    public double kGeffort;
    public double kPeffort;
    public double kIeffort;
    public double totalEffort;
    public double feedbackError;
  }

  public default void setHomePosition(double degrees) {}

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmOpenLoop(double outputVoltage) {}

  public default void requestPositionCoral(double requestedSetpoint) {}

  public default void requestPositionAlgae(double requestedSetpoint) {}

  public default void stopArmMotor() {}

  public default void setVoltage(double volts) {}

  public default void requestSlowPosition(double requestSetpoint) {}

  public default void setManualInitialization() {}

  public default void enableBrakeMode(boolean enable) {}

  public default TalonFX getKrakenFX() {
    return null;
  } // for tuning
}
