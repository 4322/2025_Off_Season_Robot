package frc.robot.constants;

import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.MinwrapConfig;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SwerveUtil.SwerveModuleConstants;
import frc.robot.util.SwerveUtil.SwerveModuleConstantsFactory;

public class DrivetrainConstants {
  // Shared drivetrain constants
  public static final int gyroId = 0;
  public static final double maxSpeedAt12Volts = 4.0; // TODO

  public static final double driveMotorKv = 800; // RPM/v
  private static final double driveGearRatio = 5.90; // SDS MK4i L2+ drive gear ratio
  private static final double turnGearRatio = 150.0 / 7.0;
  private static final double turnCoupleRatio = 3.8181818181818183; // TODO

  private static final double wheelRadius = Units.inchesToMeters(2.0);

  private static final boolean turnMotorInverted = false;
  private static final boolean turnEncoderInverted = true;
  private static final boolean invertLeftSideDrive = false;
  private static final boolean invertRightSideDrive = true;

  public static final double driveSupplyCurrentLimit = 40.0; 
  public static final double driveSupplyCurrentTime = 0.0; 
  public static final double driveStatorCurrentLimit = 75; 

  public static final double turnSupplyCurrentLimit = 20.0; 
  public static final double turnSupplyCurrentTime = 0.0; 
  public static final double turnStatorCurrentLimit = 35; 

  private static final double drivekP = 0; // TODO
  private static final double drivekD = 0; // TODO
  private static final double drivekS = 0; // TODO
  private static final double drivekV = driveMotorKv / 60.0 / driveGearRatio; // TODO

  private static final double turnkP = 150; // TODO still needs to be tuned
  private static final double turnkD = 0.2; // TODO
  private static final double turnAccelerationLimit = 1000; // TODO
  private static final double turnDeaccelerationLimit = 1000; // TODO
  private static final double turnVelocityLimit = 1000; // TODO

  private static final double simTurnInertia = 0.00001;
  private static final double simDriveInertia = 0.001;

  // Specific module constants:
  // Front Left
  private static final int frontLeftDriveMotorId = 6;
  private static final int frontLeftTurnMotorId = 4;
  private static final int frontLeftTurnEncoderId = 3;
  private static final double halfWheelBaseMeters = 0.57785;

  private static final double frontLeftXPos = halfWheelBaseMeters;
  private static final double frontLeftYPos = halfWheelBaseMeters;

  // Front Right
  private static final int frontRightDriveMotorId = 9; //
  private static final int frontRightTurnMotorId = 3; // Done
  private static final int frontRightTurnEncoderId = 4;

  private static final double frontRightXPos = halfWheelBaseMeters;
  private static final double frontRightYPos = -halfWheelBaseMeters;

  // Back Left
  private static final int backLeftDriveMotorId = 16; // Done
  private static final int backLeftTurnMotorId = 12;
  private static final int backLeftTurnEncoderId = 2;

  private static final double backLeftXPos = -halfWheelBaseMeters;
  private static final double backLeftYPos = halfWheelBaseMeters;

  // Back Right
  private static final int backRightDriveMotorId = 14; // Done
  private static final int backRightTurnMotorId = 8; // Done
  private static final int backRightTurnEncoderId = 1;

  private static final double backRightXPos = -halfWheelBaseMeters;
  private static final double backRightYPos = -halfWheelBaseMeters;

  // Auto-configured objects below
  public static final ElectricalLimitSettings driveElectricalLimitSettings =
      ElectricalLimitSettings.defaultSettings()
          .setBusCurrentLimit(driveSupplyCurrentLimit)
          .setBusCurrentLimitTime(driveSupplyCurrentTime)
          .setStatorCurrentLimit(driveStatorCurrentLimit);
  public static final ElectricalLimitSettings turnElectricalLimitSettings =
      ElectricalLimitSettings.defaultSettings()
          .setBusCurrentLimit(turnSupplyCurrentLimit)
          .setBusCurrentLimitTime(turnSupplyCurrentTime)
          .setStatorCurrentLimit(turnStatorCurrentLimit);

  private static final PIDSettings drivePIDSettings =
      PIDSettings.defaultSettings(PIDConfigSlot.kSlot0)
          .setPID(drivekP, 0, drivekD)
          .setStaticFeedforward(drivekS)
          .setVelocityFeedforward(drivekV);
  private static final PIDSettings turnPIDSettings =
      PIDSettings.defaultSettings(PIDConfigSlot.kSlot0)
          .setPID(turnkP, 0, turnkD)
          .setMotionProfileAccelLimit(turnAccelerationLimit)
          .setMotionProfileDeaccelLimit(turnDeaccelerationLimit)
          .setMotionProfileVelocityLimit(turnVelocityLimit)
          .setMinwrapConfig(new MinwrapConfig.Enabled());

  private static final SwerveModuleConstantsFactory moduleCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGains(drivePIDSettings)
          .withTurnMotorGains(turnPIDSettings)
          .withDriveElectricalLimitSettings(driveElectricalLimitSettings)
          .withTurnElectricalLimitSettings(turnElectricalLimitSettings)
          .withDriveMotorGearRatio(driveGearRatio)
          .withTurnMotorGearRatio(turnGearRatio)
          .withCouplingGearRatio(turnCoupleRatio)
          .withWheelRadius(wheelRadius)
          .withTurnMotorInverted(turnMotorInverted)
          .withTurnEncoderInverted(turnEncoderInverted)
          .withSpeedAt12Volts(maxSpeedAt12Volts)
          .withTurnMotorInverted(turnMotorInverted)
          .withTurnEncoderInverted(turnEncoderInverted)
          .withSimTurnInertia(simTurnInertia)
          .withSimDriveInertia(simDriveInertia);

  public static final SwerveModuleConstants frontLeft =
      moduleCreator.createModuleConstants(
          frontLeftTurnMotorId,
          frontLeftDriveMotorId,
          frontLeftTurnEncoderId,
          frontLeftXPos,
          frontLeftYPos,
          invertLeftSideDrive);
  public static final SwerveModuleConstants frontRight =
      moduleCreator.createModuleConstants(
          frontRightTurnMotorId,
          frontRightDriveMotorId,
          frontRightTurnEncoderId,
          frontRightXPos,
          frontRightYPos,
          invertRightSideDrive);
  public static final SwerveModuleConstants backLeft =
      moduleCreator.createModuleConstants(
          backLeftTurnMotorId,
          backLeftDriveMotorId,
          backLeftTurnEncoderId,
          backLeftXPos,
          backLeftYPos,
          invertLeftSideDrive);
  public static final SwerveModuleConstants backRight =
      moduleCreator.createModuleConstants(
          backRightTurnMotorId,
          backRightDriveMotorId,
          backRightTurnEncoderId,
          backRightXPos,
          backRightYPos,
          invertRightSideDrive);
}
