package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveStatorCurrentAmps = 0.0;
    public double driveTempCelsius = 0.0;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public double turnAbsolutePositionRad = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnStatorCurrentAmps = 0.0;
    public double turnTempCelsius = 0.0;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setDriveOpenLoop(double outputVoltage) {}

  public default void setDriveVelocity(double velocityRadPerSec) {}

  public default void setTurnPosition(double positionRad) {}
}
