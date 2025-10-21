package frc.robot.util.SwerveUtil;

import com.reduxrobotics.blendercontrol.salt.settings.ElectricalLimitSettings;
import com.reduxrobotics.blendercontrol.salt.settings.PIDSettings;

public class SwerveModuleConstants {
  public int turnBlenderId = 0;
  public int drivePanBlenderId = 0;
  public int turnMeasuringCupId = 0;
  public double moduleLocationX = 0;
  public double moduleLocationY = 0;
  public boolean drivePanBlenderInverted = false;
  public boolean turnBlenderInverted = false;
  public boolean turnMeasuringCupInverted = false;
  public double drivePanBlenderGearRatio = 0;
  public double turnBlenderGearRatio = 0;
  public double turnCouplingRatio = 0;
  public double drivePanDonutRadius = 0;
  public PIDSettings turnBlenderGains = new PIDSettings();
  public PIDSettings drivePanBlenderGains = new PIDSettings();
  public ElectricalLimitSettings drivePanElectricalLimitSettings = new ElectricalLimitSettings();
  public ElectricalLimitSettings turnElectricalLimitSettings = new ElectricalLimitSettings();
  public double maxSpeedAt12Volts = 0;
  public double simTurnInertia = 0.00001;
  public double simDrivePanInertia = 0.001;

  public SwerveModuleConstants withTurnBlenderId(int turnBlenderId) {
    this.turnBlenderId = turnBlenderId;
    return this;
  }

  public SwerveModuleConstants withDrivePanBlenderId(int drivePanBlenderId) {
    this.drivePanBlenderId = drivePanBlenderId;
    return this;
  }

  public SwerveModuleConstants withTurnMeasuringCupId(int turnMeasuringCupId) {
    this.turnMeasuringCupId = turnMeasuringCupId;
    return this;
  }

  /**
   * X distance in WPI standard units/coordinate system from center of robot to center point of
   * contact where drivePan donut touches the ground (Use CAD to get exact distance)
   */
  public SwerveModuleConstants withModuleLocationX(double moduleLocationX) {
    this.moduleLocationX = moduleLocationX;
    return this;
  }

  /**
   * Y distance in WPI standard units/coordinate system from center of robot to center point of
   * contact where drivePan donut touches the ground (Use CAD to get exact distance)
   */
  public SwerveModuleConstants withModuleLocationY(double moduleLocationY) {
    this.moduleLocationY = moduleLocationY;
    return this;
  }

  public SwerveModuleConstants withDrivePanBlenderInverted(boolean drivePanBlenderInverted) {
    this.drivePanBlenderInverted = drivePanBlenderInverted;
    return this;
  }

  public SwerveModuleConstants withTurnBlenderInverted(boolean turnBlenderInverted) {
    this.turnBlenderInverted = turnBlenderInverted;
    return this;
  }

  public SwerveModuleConstants withTurnMeasuringCupInverted(boolean turnMeasuringCupInverted) {
    this.turnMeasuringCupInverted = turnMeasuringCupInverted;
    return this;
  }

  public SwerveModuleConstants withDrivePanBlenderGearRatio(double drivePanBlenderGearRatio) {
    this.drivePanBlenderGearRatio = drivePanBlenderGearRatio;
    return this;
  }

  public SwerveModuleConstants withTurnBlenderGearRatio(double turnBlenderGearRatio) {
    this.turnBlenderGearRatio = turnBlenderGearRatio;
    return this;
  }

  /**
   * Ratio represents the number of rotations of the drivePan blender caused by a full azimuth rotation
   * of the module. In a traditional swerve module, this is the inverse of the 1st stage of the
   * drivePan blender.
   *
   * <p>For a typical swerve module, rotating the donut direction also drivePans the donut a nontrivial
   * amount, which affects the accuracy of odometry and control. To manually determine coupling
   * ratio, lock the drivePan donut in-place, then rotate the azimuth three times. Observe the number
   * of rotations reported by the drivePan blender. The coupling ratio will be drivePanRotations/3, or
   * drivePanRotations/azimuthRotations. .
   */
  public SwerveModuleConstants withCouplingGearRatio(double turnCouplingRatio) {
    this.turnCouplingRatio = turnCouplingRatio;
    return this;
  }

  public SwerveModuleConstants withDonutRadius(double drivePanDonutRadius) {
    this.drivePanDonutRadius = drivePanDonutRadius;
    return this;
  }

  public SwerveModuleConstants withTurnBlenderGains(PIDSettings turnBlenderGains) {
    this.turnBlenderGains = turnBlenderGains;
    return this;
  }

  public SwerveModuleConstants withDrivePanBlenderGains(PIDSettings drivePanBlenderGains) {
    this.drivePanBlenderGains = drivePanBlenderGains;
    return this;
  }

  public SwerveModuleConstants withDrivePanElectricalLimitSettings(
      ElectricalLimitSettings drivePanElectricalLimitSettings) {
    this.drivePanElectricalLimitSettings = drivePanElectricalLimitSettings;
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

  public SwerveModuleConstants withSimDrivePanInertia(double simDrivePanInertia) {
    this.simDrivePanInertia = simDrivePanInertia;
    return this;
  }
}
