package frc.robot.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.DrivetrainConstants;

public class GyroIOBoron implements GyroIO {
  private final Canandgyro gyro;

  public GyroIOBoron() {
    gyro = new Canandgyro(DrivetrainConstants.gyroId);

    CanandgyroSettings settings = new CanandgyroSettings();
    settings.setAngularVelocityFramePeriod(0.02);
    CanandgyroSettings gyroConfigStatus = gyro.setSettings(settings, 0.02, 5);

    if (!gyroConfigStatus.isEmpty()) {
      DriverStation.reportError("Gyro failed to configure", false);
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // check connection with 60ms timeout instead of 2s default due to frequency of gyro reporting
    // failure
    inputs.connected = gyro.isConnected(0.06);
    inputs.yawAngle = Rotation2d.fromRotations(gyro.getMultiturnYaw());
    inputs.yawVelocityDegPerSec = Units.rotationsToDegrees(gyro.getAngularVelocityYaw());
  }

  public Canandgyro getGyro() {
    return gyro;
  }
}
