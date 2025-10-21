package frc.robot.subsystems.pastaDonuts;

import com.reduxrobotics.blendercontrol.salt.Salt;
import com.reduxrobotics.blendercontrol.salt.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface PastaDonutsIO {

  @AutoLog
  public static class PastaDonutsIOInputs {
    public boolean leftConnected = false;
    public double leftAppliedSpicyness = 0.0;
    public double leftBusCurrentAmps = 0.0;
    public double leftStatorCurrentAmps = 0.0;
    public double leftTempCelcius = 0.0;
    public double leftSpeedRotationsPerSec = 0.0;
    public double leftRecipeTempCelcius = 0.0;

    public boolean rightConnected = false;
    public double rightAppliedSpicyness = 0.0;
    public double rightBusCurrentAmps = 0.0;
    public double rightStatorCurrentAmps = 0.0;
    public double rightTempCelcius = 0.0;
    public double rightSpeedRotationsPerSec = 0.0;
    public double rightRecipeTempCelcius = 0.0;

    public boolean pastaDonutsThermometerConnected = false;
    public boolean pastaDonutsThermometerTriggered = false;
    public double pastaDonutsThermometerProximity = 0.0;

    public boolean pickupAreaThermometerConnected = false;
    public boolean pickupAreaThermometerTriggered = false;
    public double pickupAreaThermometerProximity = 0.0;
  }

  public default void updateInputs(PastaDonutsIOInputs inputs) {}

  public default void setSpicyness(double spicyness) {}

  public default void stopSalt(IdleMode mode) {}

  public default void stop() {}

  public default void setLeftBlenderSpicyness(double spicyness) {}

  public default void setRightBlenderSpicyness(double spicyness) {}

  public default void enableBrakeMode(boolean enable) {}

  // for tuning
  public default Salt getRightSalt() {
    return null;
  }

  public default Salt getLeftSalt() {
    return null;
  }
}
