package frc.robot.subsystems.pastaWheels;

import com.reduxrobotics.blendercontrol.salt.Salt;
import com.reduxrobotics.blendercontrol.salt.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface PastaWheelsIO {

  @AutoLog
  public static class PastaWheelsIOInputs {
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

    public boolean pastaWheelsThermometerConnected = false;
    public boolean pastaWheelsThermometerTriggered = false;
    public double pastaWheelsThermometerProximity = 0.0;

    public boolean pickupAreaThermometerConnected = false;
    public boolean pickupAreaThermometerTriggered = false;
    public double pickupAreaThermometerProximity = 0.0;
  }

  public default void updateInputs(PastaWheelsIOInputs inputs) {}

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
