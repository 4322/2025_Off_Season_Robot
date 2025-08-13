package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionDeg = 0.0; // TODO: Add actual position
    public double heightMeters = 0.0; // TODO: Add actual height
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}
}
