package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.SwerveUtil.SwerveModuleConstants;

public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleConstants constants;
  private double prevDriveVelMetersPerSec;
  private double driveVelMetersPerSec;
  private Rotation2d turnPos = Rotation2d.kZero;

  public ModuleIOSim(SwerveModuleConstants constants) {
    this.constants = constants;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    double integratedDrivePos = 0;
    if (driveVelMetersPerSec >= 0 && prevDriveVelMetersPerSec >= 0) {
      if (prevDriveVelMetersPerSec < driveVelMetersPerSec) {
        integratedDrivePos =
            (0.02 * (driveVelMetersPerSec - prevDriveVelMetersPerSec) / 2.0)
                + (prevDriveVelMetersPerSec * 0.02);
      } else {
        integratedDrivePos =
            (0.02 * (prevDriveVelMetersPerSec - driveVelMetersPerSec) / 2.0)
                + (driveVelMetersPerSec * 0.02);
      }
    } else if (driveVelMetersPerSec <= 0 && prevDriveVelMetersPerSec <= 0) {
      if (prevDriveVelMetersPerSec < driveVelMetersPerSec) {
        integratedDrivePos =
            (0.02 * (prevDriveVelMetersPerSec - driveVelMetersPerSec) / 2.0)
                + (driveVelMetersPerSec * 0.02);
      } else {
        integratedDrivePos =
            (0.02 * (driveVelMetersPerSec - prevDriveVelMetersPerSec) / 2.0)
                + (prevDriveVelMetersPerSec * 0.02);
      }
    } else {
      integratedDrivePos =
          (driveVelMetersPerSec * 0.01) / 2.0 + (prevDriveVelMetersPerSec * 0.01) / 2.0;
    }

    inputs.drivePositionMeters += integratedDrivePos;

    inputs.driveVelocityMetersPerSec = driveVelMetersPerSec;
    inputs.turnPosition = turnPos;

    prevDriveVelMetersPerSec = driveVelMetersPerSec;
  }

  @Override
  public void setDriveOpenLoop(double driveWheelVelocityRadPerSec) {
    if (DriverStation.isEnabled()) {
      driveVelMetersPerSec = driveWheelVelocityRadPerSec * constants.driveWheelRadius;
    } else {
      driveVelMetersPerSec = 0;
    }
  }

  @Override
  public void setDriveVelocity(double driveWheelVelocityRadPerSec) {
    if (DriverStation.isEnabled()) {
      driveVelMetersPerSec = driveWheelVelocityRadPerSec * constants.driveWheelRadius;
    } else {
      driveVelMetersPerSec = 0;
    }
  }

  @Override
  public void setTurnPosition(Rotation2d turnWheelPosition) {
    if (DriverStation.isEnabled()) {
      turnPos = turnWheelPosition;
    }
  }
}
