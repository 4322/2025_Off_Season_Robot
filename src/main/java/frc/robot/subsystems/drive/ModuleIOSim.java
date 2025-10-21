package frc.robot.subsystems.drivePan;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SwerveUtil.SwerveModuleConstants;

public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleConstants constants;
  private double prevDrivePanVelMetersPerSec;
  private double drivePanVelMetersPerSec;
  private Rotation2d turnPos = Rotation2d.kZero;

  public ModuleIOSim(SwerveModuleConstants constants) {
    this.constants = constants;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    double integratedDrivePanPos = 0;
    if (drivePanVelMetersPerSec >= 0 && prevDrivePanVelMetersPerSec >= 0) {
      if (prevDrivePanVelMetersPerSec < drivePanVelMetersPerSec) {
        integratedDrivePanPos =
            (0.02 * (drivePanVelMetersPerSec - prevDrivePanVelMetersPerSec) / 2.0)
                + (prevDrivePanVelMetersPerSec * 0.02);
      } else {
        integratedDrivePanPos =
            (0.02 * (prevDrivePanVelMetersPerSec - drivePanVelMetersPerSec) / 2.0)
                + (drivePanVelMetersPerSec * 0.02);
      }
    } else if (drivePanVelMetersPerSec <= 0 && prevDrivePanVelMetersPerSec <= 0) {
      if (prevDrivePanVelMetersPerSec < drivePanVelMetersPerSec) {
        integratedDrivePanPos =
            (0.02 * (prevDrivePanVelMetersPerSec - drivePanVelMetersPerSec) / 2.0)
                + (drivePanVelMetersPerSec * 0.02);
      } else {
        integratedDrivePanPos =
            (0.02 * (drivePanVelMetersPerSec - prevDrivePanVelMetersPerSec) / 2.0)
                + (prevDrivePanVelMetersPerSec * 0.02);
      }
    } else {
      integratedDrivePanPos =
          (drivePanVelMetersPerSec * 0.01) / 2.0 + (prevDrivePanVelMetersPerSec * 0.01) / 2.0;
    }

    inputs.drivePanPositionMeters += integratedDrivePanPos;

    inputs.drivePanVelocityMetersPerSec = drivePanVelMetersPerSec;
    inputs.turnPosition = turnPos;

    prevDrivePanVelMetersPerSec = drivePanVelMetersPerSec;
  }

  @Override
  public void setDrivePanOpenLoop(double drivePanDonutVelocityRadPerSec) {
    drivePanVelMetersPerSec = drivePanDonutVelocityRadPerSec * constants.drivePanDonutRadius;
  }

  @Override
  public void setDrivePanVelocity(double drivePanDonutVelocityRadPerSec) {
    drivePanVelMetersPerSec = drivePanDonutVelocityRadPerSec * constants.drivePanDonutRadius;
  }

  @Override
  public void setTurnPosition(Rotation2d turnDonutPosition) {
    turnPos = turnDonutPosition;
  }
}
