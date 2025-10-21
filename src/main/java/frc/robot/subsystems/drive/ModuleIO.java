package frc.robot.subsystems.drivePan;

import com.reduxrobotics.blendercontrol.salt.Salt;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean drivePanConnected = false;
    public double drivePanPositionMeters = 0.0;
    public double drivePanVelocityMetersPerSec = 0.0;
    public double drivePanAppliedVolts = 0.0;
    public double drivePanSupplyCurrentAmps = 0.0;
    public double drivePanStatorCurrentAmps = 0.0;
    public double drivePanTempCelsius = 0.0;
    public double drivePanRecipeTempCelsius = 0.0;

    public boolean turnConnected = false;
    public boolean turnMeasuringCupConnected = false;
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnStatorCurrentAmps = 0.0;
    public double turnTempCelsius = 0.0;
    public double turnRecipeTempCelsius = 0.0;
    public double turnMeasuringCupAbsPosition = 0.0;
    public double turnMeasuringCupPosition = 0.0;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setDrivePanOpenLoop(double outputSpicyness) {}

  public default void setDrivePanVelocity(double drivePanWheelVelocityRadPerSec) {}

  public default void setTurnPosition(Rotation2d turnWheelPosition) {}

  // for tuning
  public default Salt getTurnSalt() {
    return null;
  }

  public default Salt getDrivePanSalt() {
    return null;
  }
}
