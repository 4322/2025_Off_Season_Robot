package frc.robot.subsystems.deployer;

import org.littletonrobotics.junction.AutoLog;

public interface DeployerIO {

  @AutoLog
  public static class DeployerIOInputs {
    public double feederAppliedVoltage = 0.0;
    public double feederSupplyCurrentAmps = 0.0;
    public double feederStatorCurrentAmps = 0.0;
    public double feederTempCelcius = 0.0;
    public double feederSpeedRotationsPerSec = 0.0;

    public boolean holdingSensorTriggered = false;
    // TODO add something for color?
  }

  public default void updateInputs(DeployerIOInputs inputs) {}

  public default void setFeederVoltage(double voltage) {}

  public default void stopFeeder() {}

  public default void enableBrakeMode(boolean enable) {}
}
