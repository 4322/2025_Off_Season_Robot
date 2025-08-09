package frc.robot.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DrivetrainConstants;

public class GyroIOBoron implements GyroIO {
  private final Canandgyro gyro;

  public GyroIOBoron() {
    gyro = new Canandgyro(DrivetrainConstants.gyroID);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.yawAngleRad = Units.rotationsToRadians(gyro.getYaw());
    inputs.yawVelocityRadPerSec = Units.rotationsToRadians(gyro.getAngularVelocityYaw());
  }

  @Override
  public Canandgyro getGyro() {
    return gyro;
  }
}
