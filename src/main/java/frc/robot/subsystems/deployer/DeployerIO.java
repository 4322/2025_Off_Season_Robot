package frc.robot.subsystems.deployer;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface DeployerIO {

  @AutoLog
  public static class DeployerIOInputs {

    public boolean connected = false;
    public double statorCurrentAmps = 0.0;
    public double busCurrentAmps = 0.0;
    public double tempCelcius = 0.0;
    public double speedRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double angleDeg = 0.0;

    public double deployerMotorMechanismPositionDegrees = 0.0;

    public double prevRequestedPositionDeg = 0.0;
  }

  public default void updateInputs(DeployerIOInputs inputs) {}

  public default void setPosition(double rotations) {}

  public default void stop(IdleMode idleMode) {}

  public default void setHome() {}

  public default void setVoltage(double voltage) {}
}
