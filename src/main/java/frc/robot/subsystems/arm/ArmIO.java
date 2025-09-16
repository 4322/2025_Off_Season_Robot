package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double requestedPosDeg;
    public double voltage = 0.0;
    public double velocityDegSec = 0.0;
    public boolean armConnected = false;
    public boolean armEncoderConnected = false;
    public double armSupplyCurrentAmps = 0.0;
    public double armStatorCurrentAmps = 0.0;
    public double armTempCelsius = 0.0;
    public double armPositionDegrees =
        0.0; // 0 is vertical to front of robot. Posititve clockwise looking from the left
  }

  public default void setHomePosition() {}

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmOpenLoop(double outputVoltage) {}

  public default void requestPosition(double requestedSetpoint) {}

  public default void stopArmMotor(IdleMode idlemode) {}

  public default void setVoltage(double volts) {}

  public default void requestSlowPosition(double requestSetpoint) {}

  public default void setManualInitialization() {}
}
