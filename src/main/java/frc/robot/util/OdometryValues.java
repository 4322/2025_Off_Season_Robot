package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class OdometryValues {
  public Rotation2d rotation;
  public double timestamp;
  public SwerveModulePosition[] modulePositions;

  public OdometryValues(
      double timestamp, Rotation2d rotation, SwerveModulePosition[] modulePositions) {
    this.rotation = rotation;
    this.timestamp = timestamp;
    this.modulePositions = modulePositions;
  }
}
