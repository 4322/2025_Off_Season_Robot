package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double positionDeg = 0.0;
    public double velocityDegPerSec = 0.0;
    public double appliedVolts = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
