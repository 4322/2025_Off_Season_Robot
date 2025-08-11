package frc.robot.util.SwerveUtil;

import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;

public class SwerveModuleConstantsFactory {
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

  public SwerveModuleConstantsFactory withTurnMotorInverted(boolean turnMotorInverted) {
    this.turnMotorInverted = turnMotorInverted;
    return this;
  }

  public SwerveModuleConstantsFactory withTurnEncoderInverted(boolean turnEncoderInverted) {
    this.turnEncoderInverted = turnEncoderInverted;
    return this;
  }

  public SwerveModuleConstantsFactory withDriveMotorGearRatio(double driveMotorGearRatio) {
    this.driveMotorGearRatio = driveMotorGearRatio;
    return this;
  }

  public SwerveModuleConstantsFactory withTurnMotorGearRatio(double turnMotorGearRatio) {
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
  public SwerveModuleConstantsFactory withCouplingGearRatio(double turnCouplingRatio) {
    this.turnCouplingRatio = turnCouplingRatio;
    return this;
  }

  public SwerveModuleConstantsFactory withWheelRadius(double driveWheelRadius) {
    this.driveWheelRadius = driveWheelRadius;
    return this;
  }

  public SwerveModuleConstantsFactory withTurnMotorGains(PIDSettings turnMotorGains) {
    this.turnMotorGains = turnMotorGains;
    return this;
  }

  public SwerveModuleConstantsFactory withDriveMotorGains(PIDSettings driveMotorGains) {
    this.driveMotorGains = driveMotorGains;
    return this;
  }

  public SwerveModuleConstantsFactory withDriveElectricalLimitSettings(
      ElectricalLimitSettings driveElectricalLimitSettings) {
    this.driveElectricalLimitSettings = driveElectricalLimitSettings;
    return this;
  }

  public SwerveModuleConstantsFactory withTurnElectricalLimitSettings(
      ElectricalLimitSettings turnElectricalLimitSettings) {
    this.turnElectricalLimitSettings = turnElectricalLimitSettings;
    return this;
  }

  public SwerveModuleConstantsFactory withSpeedAt12Volts(double speedAt12Volts) {
    this.maxSpeedAt12Volts = speedAt12Volts;
    return this;
  }

  public SwerveModuleConstantsFactory withSimTurnInertia(double simTurnInertia) {
    this.simTurnInertia = simTurnInertia;
    return this;
  }

  public SwerveModuleConstantsFactory withSimDriveInertia(double simDriveInertia) {
    this.simDriveInertia = simDriveInertia;
    return this;
  }

  /**
   * @param steerMotorId CAN ID of the steer motor.
   * @param driveMotorId CAN ID of the drive motor.
   * @param turnEncoderId CAN ID of the absolute encoder used for azimuth.
   * @param moduleLocationX The location of this module's wheels relative to the physical center of
   *     the robot in meters along the X axis of the robot.
   * @param moduleLocationY The location of this module's wheels relative to the physical center of
   *     the robot in meters along the Y axis of the robot.
   * @param driveMotorInverted True if the drive motor is inverted.
   * @param turnMotorInverted True if the steer motor is inverted from the azimuth. The azimuth
   *     should rotate counter-clockwise (as seen from the top of the robot) for a positive motor
   *     output.
   * @param turnEncoderInverted True if the turn encoder is inverted from the azimuth. The encoder
   *     should report a positive velocity when the azimuth rotates counter-clockwise (as seen from
   *     the top of the robot).
   * @return Constants for the swerve module
   */
  public SwerveModuleConstants createModuleConstants(
      int steerMotorId,
      int driveMotorId,
      int turnEncoderId,
      double moduleLocationX,
      double moduleLocationY,
      boolean driveMotorInverted) {
    return new SwerveModuleConstants()
        .withTurnMotorId(steerMotorId)
        .withDriveMotorId(driveMotorId)
        .withTurnEncoderId(turnEncoderId)
        .withModuleLocationX(moduleLocationX)
        .withModuleLocationY(moduleLocationY)
        .withDriveMotorInverted(driveMotorInverted)
        .withTurnMotorInverted(turnMotorInverted)
        .withTurnEncoderInverted(turnEncoderInverted)
        .withDriveMotorGearRatio(driveMotorGearRatio)
        .withTurnMotorGearRatio(turnMotorGearRatio)
        .withCouplingGearRatio(turnCouplingRatio)
        .withWheelRadius(driveWheelRadius)
        .withTurnMotorGains(turnMotorGains)
        .withDriveMotorGains(driveMotorGains)
        .withDriveElectricalLimitSettings(driveElectricalLimitSettings)
        .withTurnElectricalLimitSettings(turnElectricalLimitSettings)
        .withSpeedAt12Volts(maxSpeedAt12Volts)
        .withSimTurnInertia(simTurnInertia)
        .withSimDriveInertia(simDriveInertia);
  }
}
