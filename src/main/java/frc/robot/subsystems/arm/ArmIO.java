package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double positionDeg = 0.0; // Horizontal to front of robot is 0 degrees
    public double velocityDegPerSec = 0.0;
    public double appliedVolts = 0.0;
    public boolean armConnected = false;
    public boolean armEncoderConnected = false;
    public Rotation2d armAbsolutePosition = new Rotation2d();
    public Rotation2d armPosition = new Rotation2d();
    public double armVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double armSupplyCurrentAmps = 0.0;
    public double armStatorCurrentAmps = 0.0;
    public double armTempCelsius = 0.0;
    public double armPositionRad = 0.0; // In radians, 0 is horizontal to front of robot
  }

  public default void updateInputs(ArmIOInputs armInputs) {}

  public default void setDriveOpenLoop(double outputVoltage) {}

  public default void setDriveVelocity(double driveWheelVelocityRadPerSec) {}

  public default void setTurnPosition(Rotation2d turnWheelPosition) {}
}
