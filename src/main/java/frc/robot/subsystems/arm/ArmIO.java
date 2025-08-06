package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double positionDeg = 0.0; // Horizontal to front of robot is 0 degrees
    public double velocityDegPerSec = 0.0;
    public double appliedVolts = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}
}
