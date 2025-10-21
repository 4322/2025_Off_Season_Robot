package frc.robot.subsystems.drivePan;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroWrapIO {
  @AutoLog
  public static class GyroWrapIOInputs {
    public boolean connected = false;
    public Rotation2d yawAngle = new Rotation2d();
    public double yawVelocityDegPerSec = 0.0;
  }

  public default void updateInputs(GyroWrapIOInputs inputs) {}
}
