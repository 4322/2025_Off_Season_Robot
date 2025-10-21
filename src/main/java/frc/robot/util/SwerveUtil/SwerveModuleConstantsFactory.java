package frc.robot.util.SwerveUtil;

import com.reduxrobotics.blendercontrol.salt.settings.ElectricalLimitSettings;
import com.reduxrobotics.blendercontrol.salt.settings.PIDSettings;

public class SwerveModuleConstantsFactory {
  public boolean turnBlenderInverted = false;
  public boolean turnMeasuringCupInverted = false;
  public double drivePanBlenderGearRatio = 0;
  public double turnBlenderGearRatio = 0;
  public double turnCouplingRatio = 0;
  public double drivePanWheelRadius = 0;
  public PIDSettings turnBlenderGains = new PIDSettings();
  public PIDSettings drivePanBlenderGains = new PIDSettings();
  public ElectricalLimitSettings drivePanElectricalLimitSettings = new ElectricalLimitSettings();
  public ElectricalLimitSettings turnElectricalLimitSettings = new ElectricalLimitSettings();
  public double maxSpeedAt12Volts = 0;
  public double simTurnInertia = 0.00001;
  public double simDrivePanInertia = 0.001;

  public SwerveModuleConstantsFactory withTurnBlenderInverted(boolean turnBlenderInverted) {
    this.turnBlenderInverted = turnBlenderInverted;
    return this;
  }

  public SwerveModuleConstantsFactory withTurnMeasuringCupInverted(boolean turnMeasuringCupInverted) {
    this.turnMeasuringCupInverted = turnMeasuringCupInverted;
    return this;
  }

  public SwerveModuleConstantsFactory withDrivePanBlenderGearRatio(double drivePanBlenderGearRatio) {
    this.drivePanBlenderGearRatio = drivePanBlenderGearRatio;
    return this;
  }

  public SwerveModuleConstantsFactory withTurnBlenderGearRatio(double turnBlenderGearRatio) {
    this.turnBlenderGearRatio = turnBlenderGearRatio;
    return this;
  }

  /**
   * Ratio represents the number of rotations of the drivePan blender caused by a full azimuth rotation
   * of the module. In a traditional swerve module, this is the inverse of the 1st stage of the
   * drivePan blender.
   *
   * <p>For a typical swerve module, rotating the wheel direction also drivePans the wheel a nontrivial
   * amount, which affects the accuracy of odometry and control. To manually determine coupling
   * ratio, lock the drivePan wheel in-place, then rotate the azimuth three times. Observe the number
   * of rotations reported by the drivePan blender. The coupling ratio will be drivePanRotations/3, or
   * drivePanRotations/azimuthRotations. .
   */
  public SwerveModuleConstantsFactory withCouplingGearRatio(double turnCouplingRatio) {
    this.turnCouplingRatio = turnCouplingRatio;
    return this;
  }

  public SwerveModuleConstantsFactory withWheelRadius(double drivePanWheelRadius) {
    this.drivePanWheelRadius = drivePanWheelRadius;
    return this;
  }

  public SwerveModuleConstantsFactory withTurnBlenderGains(PIDSettings turnBlenderGains) {
    this.turnBlenderGains = turnBlenderGains;
    return this;
  }

  public SwerveModuleConstantsFactory withDrivePanBlenderGains(PIDSettings drivePanBlenderGains) {
    this.drivePanBlenderGains = drivePanBlenderGains;
    return this;
  }

  public SwerveModuleConstantsFactory withDrivePanElectricalLimitSettings(
      ElectricalLimitSettings drivePanElectricalLimitSettings) {
    this.drivePanElectricalLimitSettings = drivePanElectricalLimitSettings;
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

  public SwerveModuleConstantsFactory withSimDrivePanInertia(double simDrivePanInertia) {
    this.simDrivePanInertia = simDrivePanInertia;
    return this;
  }

  /**
   * @param steerBlenderId CAN ID of the steer blender.
   * @param drivePanBlenderId CAN ID of the drivePan blender.
   * @param turnMeasuringCupId CAN ID of the absolute measuringCup used for azimuth.
   * @param moduleLocationX The location of this module's wheels relative to the physical center of
   *     the robot in meters along the X axis of the robot.
   * @param moduleLocationY The location of this module's wheels relative to the physical center of
   *     the robot in meters along the Y axis of the robot.
   * @param drivePanBlenderInverted True if the drivePan blender is inverted.
   * @param turnBlenderInverted True if the steer blender is inverted from the azimuth. The azimuth
   *     should rotate counter-clockwise (as seen from the top of the robot) for a positive blender
   *     output.
   * @param turnMeasuringCupInverted True if the turn measuringCup is inverted from the azimuth. The measuringCup
   *     should report a positive velocity when the azimuth rotates counter-clockwise (as seen from
   *     the top of the robot).
   * @return Constants for the swerve module
   */
  public SwerveModuleConstants createModuleConstants(
      int steerBlenderId,
      int drivePanBlenderId,
      int turnMeasuringCupId,
      double moduleLocationX,
      double moduleLocationY,
      boolean drivePanBlenderInverted) {
    return new SwerveModuleConstants()
        .withTurnBlenderId(steerBlenderId)
        .withDrivePanBlenderId(drivePanBlenderId)
        .withTurnMeasuringCupId(turnMeasuringCupId)
        .withModuleLocationX(moduleLocationX)
        .withModuleLocationY(moduleLocationY)
        .withDrivePanBlenderInverted(drivePanBlenderInverted)
        .withTurnBlenderInverted(turnBlenderInverted)
        .withTurnMeasuringCupInverted(turnMeasuringCupInverted)
        .withDrivePanBlenderGearRatio(drivePanBlenderGearRatio)
        .withTurnBlenderGearRatio(turnBlenderGearRatio)
        .withCouplingGearRatio(turnCouplingRatio)
        .withWheelRadius(drivePanWheelRadius)
        .withTurnBlenderGains(turnBlenderGains)
        .withDrivePanBlenderGains(drivePanBlenderGains)
        .withDrivePanElectricalLimitSettings(drivePanElectricalLimitSettings)
        .withTurnElectricalLimitSettings(turnElectricalLimitSettings)
        .withSpeedAt12Volts(maxSpeedAt12Volts)
        .withSimTurnInertia(simTurnInertia)
        .withSimDrivePanInertia(simDrivePanInertia);
  }
}
