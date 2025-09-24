package frc.robot.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DrivetrainConstants;

public class GyroIOBoron implements GyroIO {
  private final Canandgyro gyro;

  public GyroIOBoron() {
    gyro = new Canandgyro(DrivetrainConstants.gyroId);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.yawAngle = Rotation2d.fromRotations(gyro.getMultiturnYaw());
    inputs.yawVelocityRadPerSec = Units.rotationsToRadians(gyro.getAngularVelocityYaw());
  }

  public Canandgyro getGyro() {
    return gyro;
  }
}
