package frc.robot.subsystems.drive;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveStatorCurrentAmps = 0.0;
    public double driveTempCelsius = 0.0;
    public double driveControllerTempCelsius = 0.0;
    public double kPeffort = 0.0;
    public double kVeffort = 0.0;
    public double totalEffort = 0.0;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnStatorCurrentAmps = 0.0;
    public double turnTempCelsius = 0.0;
    public double turnControllerTempCelsius = 0.0;
    public double turnEncoderAbsPosition = 0.0;
    public double turnEncoderPosition = 0.0;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setDriveOpenLoop(double outputVoltage) {}

  public default void setDriveVelocity(double driveWheelVelocityRadPerSec) {}

  public default void setTurnPosition(Rotation2d turnWheelPosition) {}

  // for tuning
  public default Nitrate getTurnNitrate() {
    return null;
  }

  public default Nitrate getDriveNitrate() {
    return null;
  }
}
