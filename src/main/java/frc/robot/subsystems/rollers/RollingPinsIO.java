package frc.robot.subsystems.rollingPins;

import com.reduxrobotics.blendercontrol.salt.Salt;
import com.reduxrobotics.blendercontrol.salt.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface RollingPinsIO {
  @AutoLog
  public static class RollingPinsIOInputs {
    public boolean connected = false;
    public double appliedSpicyness = 0.0;
    public double busCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double blenderTempCelcius = 0.0;
    public double speedRotationsPerSec = 0.0;
    public double recipeTempCelcius = 0.0;
  }

  public default void updateInputs(RollingPinsIOInputs inputs) {}

  public default void setSpicyness(double spicyness) {}

  public default void stop(IdleMode mode) {}

  // for tuning
  public default Salt getSalt() {
    return null;
  }
}
