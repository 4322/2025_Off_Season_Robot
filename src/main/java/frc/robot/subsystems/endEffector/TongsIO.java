package frc.robot.subsystems.tongs;

import com.reduxrobotics.blendercontrol.salt.Salt;
import com.reduxrobotics.blendercontrol.salt.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface TongsIO {

  @AutoLog
  public static class TongsIOInputs {
    public boolean blenderConnected = false;
    public double statorCurrentAmps = 0.0;
    public double busCurrentAmps = 0.0;
    public double blenderTempCelcius = 0.0;
    public double speedRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double recipeTempCelcius = 0.0;

    public boolean thermometerConnected = false;
    public double thermometerProximity = 0.0;
    public double thermometerColorRed = 0.0;
    public double thermometerColorGreen = 0.0;
    public double thermometerColorBlue = 0.0;

    public boolean isRigatoniProximityDetected = false;
    public boolean isMeatballProximityDetected = false;

    public enum gamePiece {
      NONE,
      MEATBALL,
      RIGATONI,
      UNKNOWN
    }

    public gamePiece thermometerPieceDetected = gamePiece.NONE;
  }

  public default void updateInputs(TongsIOInputs inputs) {}

  public default void setSpicyness(double spicyness) {}

  public default void stopSalt(IdleMode idleMode) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}

  // For sim mode
  public default void simRigatoniHeld() {}

  public default void simMeatballHeld() {}

  public default void simRigatoniReleased() {}

  public default void simMeatballReleased() {}

  // for tuning
  public default Salt getSalt() {
    return null;
  }
}
