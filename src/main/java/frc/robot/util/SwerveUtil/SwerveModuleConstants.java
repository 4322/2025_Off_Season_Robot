package frc.robot.util.SwerveUtil;

import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;

public class SwerveModuleConstants {
  public int turnMotorId = 0;
  public int driveMotorId = 0;
  public int turnEncoderId = 0;
  public double moduleLocationX = 0;
  public double moduleLocationY = 0;
  public boolean driveMotorInverted = false;
  public boolean turnMotorInverted = false;
  public boolean turnEncoderInverted = false;
  public double driveMotorGearRatio = 0;
  public double turnMotorGearRatio = 0;
  public double turnCouplingRatio = 0;
  public double driveWheelRadius = 0;
  public PIDSettings turnMotorGains = new PIDSettings();
  public PIDSettings driveMotorGains = new PIDSettings();
  public ElectricalLimitSettings driveElectricalLimitSettings = new ElectricalLimitSettings();
  public ElectricalLimitSettings turnElectricalLimitSettings = new ElectricalLimitSettings();
  public double maxSpeedAt12Volts = 0;
  public double simTurnInertia = 0.00001;
  public double simDriveInertia = 0.001;

  public SwerveModuleConstants withTurnMotorId(int turnMotorId) {
    this.turnMotorId = turnMotorId;
    return this;
  }

  public SwerveModuleConstants withDriveMotorId(int driveMotorId) {
    this.driveMotorId = driveMotorId;
    return this;
  }

  public SwerveModuleConstants withTurnEncoderId(int turnEncoderId) {
    this.turnEncoderId = turnEncoderId;
    return this;
  }

  /**
   * X distance in WPI standard units/coordinate system from center of robot to center point of
   * contact where drive wheel touches the ground (Use CAD to get exact distance)
   */
  public SwerveModuleConstants withModuleLocationX(double moduleLocationX) {
    this.moduleLocationX = moduleLocationX;
    return this;
  }

  /**
   * Y distance in WPI standard units/coordinate system from center of robot to center point of
   * contact where drive wheel touches the ground (Use CAD to get exact distance)
   */
  public SwerveModuleConstants withModuleLocationY(double moduleLocationY) {
    this.moduleLocationY = moduleLocationY;
    return this;
  }

  public SwerveModuleConstants withDriveMotorInverted(boolean driveMotorInverted) {
    this.driveMotorInverted = driveMotorInverted;
    return this;
  }

  public SwerveModuleConstants withTurnMotorInverted(boolean turnMotorInverted) {
    this.turnMotorInverted = turnMotorInverted;
    return this;
  }

  public SwerveModuleConstants withTurnEncoderInverted(boolean turnEncoderInverted) {
    this.turnEncoderInverted = turnEncoderInverted;
    return this;
  }

  public SwerveModuleConstants withDriveMotorGearRatio(double driveMotorGearRatio) {
    this.driveMotorGearRatio = driveMotorGearRatio;
    return this;
  }

  public SwerveModuleConstants withTurnMotorGearRatio(double turnMotorGearRatio) {
    this.turnMotorGearRatio = turnMotorGearRatio;
    return this;
  }

  /**
   * Ratio represents the number of rotations of the drive motor caused by a full azimuth rotation
   * of the module. In a traditional swerve module, this is the inverse of the 1st stage of the
   * drive motor.
   *
   * <p>For a typical swerve module, rotating the wheel direction also drives the wheel a nontrivial
   * amount, which affects the accuracy of odometry and control. To manually determine coupling
   * ratio, lock the drive wheel in-place, then rotate the azimuth three times. Observe the number
   * of rotations reported by the drive motor. The coupling ratio will be driveRotations/3, or
   * driveRotations/azimuthRotations. .
   */
  public SwerveModuleConstants withCouplingGearRatio(double turnCouplingRatio) {
    this.turnCouplingRatio = turnCouplingRatio;
    return this;
  }

  public SwerveModuleConstants withWheelRadius(double driveWheelRadius) {
    this.driveWheelRadius = driveWheelRadius;
    return this;
  }

  public SwerveModuleConstants withTurnMotorGains(PIDSettings turnMotorGains) {
    this.turnMotorGains = turnMotorGains;
    return this;
  }

  public SwerveModuleConstants withDriveMotorGains(PIDSettings driveMotorGains) {
    this.driveMotorGains = driveMotorGains;
    return this;
  }

  public SwerveModuleConstants withDriveElectricalLimitSettings(
      ElectricalLimitSettings driveElectricalLimitSettings) {
    this.driveElectricalLimitSettings = driveElectricalLimitSettings;
    return this;
  }

  public SwerveModuleConstants withTurnElectricalLimitSettings(
      ElectricalLimitSettings turnElectricalLimitSettings) {
    this.turnElectricalLimitSettings = turnElectricalLimitSettings;
    return this;
  }

  public SwerveModuleConstants withSpeedAt12Volts(double speedAt12Volts) {
    this.maxSpeedAt12Volts = speedAt12Volts;
    return this;
  }

  public SwerveModuleConstants withSimTurnInertia(double simTurnInertia) {
    this.simTurnInertia = simTurnInertia;
    return this;
  }

  public SwerveModuleConstants withSimDriveInertia(double simDriveInertia) {
    this.simDriveInertia = simDriveInertia;
    return this;
  }
}
