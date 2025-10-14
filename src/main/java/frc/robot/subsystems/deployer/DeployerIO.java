package frc.robot.subsystems.deployer;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface DeployerIO {

  @AutoLog
  public static class DeployerIOInputs {

    public boolean connected = false;
    public double statorCurrentAmps = 0.0;
    public double busCurrentAmps = 0.0;
    public double motorTempCelcius = 0.0;
    public double speedRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double angleDeg = 0.0;
    public double encoderRotations = 0.0;
    public double kGeffort;
    public double kPeffort;
    public double totalEffort;
    public double controllerTempCelcius = 0.0;

    public double requestedPosDeg = 0.0;
  }

  public default void updateInputs(DeployerIOInputs inputs) {}

  public default void setPosition(double rotations) {}

  public default void setPositionSlot0(double rotations) {}

  public default void stop(IdleMode idleMode) {}

  public default void setHome() {}

  public default void setVoltage(double voltage) {}

  public default Nitrate getNitrate() {
    return null;
  } // for tuning
}
