package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double requestedPosDeg;
    public double voltage = 0.0;
    public double velocityDegSec = 0.0;
    public double encoderRotations = 0.0;
    public boolean armConnected = false;
    public boolean armEncoderConnected = false;
    public double SupplyCurrentAmps = 0.0;
    public double StatorCurrentAmps = 0.0;
    public double TempCelsius = 0.0;
    public double PositionDegrees =
        0.0; // 0 is vertical to front of robot. Posititve clockwise looking from the left
    public double kGeffort;
    public double kPeffort;
    public double totalEffort;
  }

  public default void setHomePosition() {}

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmOpenLoop(double outputVoltage) {}

  public default void requestPosition(double requestedSetpoint) {}

  public default void stopArmMotor(IdleMode idlemode) {}

  public default void setVoltage(double volts) {}

  public default void requestSlowPosition(double requestSetpoint) {}

  public default void setManualInitialization() {}

  public default Nitrate getNitrate() {
    return null;
  } // for tuning
}
