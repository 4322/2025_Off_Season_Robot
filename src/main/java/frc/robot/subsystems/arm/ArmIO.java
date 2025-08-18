package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double velocityDegPerSec = 0.0;
    public double appliedVolts = 0.0;
    public boolean armConnected = false;
    public boolean armEncoderConnected = false;
    public double armPosition;
    public double armVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double armSupplyCurrentAmps = 0.0;
    public double armStatorCurrentAmps = 0.0;
    public double armTempCelsius = 0.0;
    public double armPositionDegrees = 0.0; // In radians, 0 is horizontal to front of robot
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmOpenLoop(double outputVoltage) {}

  public default void requestPosition(Rotation2d ArmPosition) {}
}
