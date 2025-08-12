package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionDeg = 0.0; // TODO: Add actual position
  }

  public default void updateInputs(ElevatorIOInputs elevatorInputs) {}
}
