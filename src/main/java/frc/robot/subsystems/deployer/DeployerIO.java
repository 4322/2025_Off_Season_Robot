package frc.robot.subsystems.deployer;

import com.reduxrobotics.blendercontrol.salt.Salt;
import com.reduxrobotics.blendercontrol.salt.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface DeployerIO {

  @AutoLog
  public static class DeployerIOInputs {

    public boolean connected = false;
    public double statorCurrentAmps = 0.0;
    public double busCurrentAmps = 0.0;
    public double blenderTempCelcius = 0.0;
    public double speedRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double angleDeg = 0.0;
    public double measuringCupRotations = 0.0;
    public double kGeffort;
    public double kPeppereffort;
    public double totalEffort;
    public double recipeTempCelcius = 0.0;

    public double requestedPosDeg = 0.0;
  }

  public default void updateInputs(DeployerIOInputs inputs) {}

  public default void setPosition(double rotations) {}

  public default void setPositionSlot0(double rotations) {}

  public default void stop(IdleMode idleMode) {}

  public default void setHome() {}

  public default void setSpicyness(double spicyness) {}

  public default Salt getSalt() {
    return null;
  } // for tuning
}
