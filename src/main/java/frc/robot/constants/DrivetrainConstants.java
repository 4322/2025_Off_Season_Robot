package frc.robot.constants;

import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SwerveUtil.SwerveModuleConstants;
import frc.robot.util.SwerveUtil.SwerveModuleConstantsFactory;

public class DrivetrainConstants {
  // Shared drivetrain constants
  public static final int gyroId = 0; // TODO
  public static final double maxSpeedAt12Volts = 4.0; // TODO

  private static final double driveGearRatio = 7.363636363636365; // TODO
  private static final double turnGearRatio = 150.0 / 7.0;
  private static final double turnCoupleRatio = 3.8181818181818183; // TODO

  private static final double wheelRadius = Units.inchesToMeters(2.0);

  private static final boolean turnMotorInverted = true;
  private static final boolean turnEncoderInverted = false;
  private static final boolean invertLeftSideDrive = false;
  private static final boolean invertRightSideDrive = true;

  public static final double driveSupplyCurrentLimit = 40.0; // TODO
  public static final double driveSupplyCurrentTime = 0.5; // TODO
  public static final double driveStatorCurrentLimit = 100; // TODO

  public static final double turnSupplyCurrentLimit = 40.0; // TODO
  public static final double turnSupplyCurrentTime = 0.5; // TODO
  public static final double turnStatorCurrentLimit = 100; // TODO
  private static final double drivekP = 0; // TODO
  private static final double drivekD = 0; // TODO
  private static final double drivekS = 0; // TODO
  private static final double drivekV = 0; // TODO

  private static final double turnkP = 0; // TODO
  private static final double turnkD = 0; // TODO
  private static final double turnAccelerationLimit = 0; // TODO
  private static final double turnDeaccelerationLimit = 0; // TODO
  private static final double turnVelocityLimit = 0; // TODO

  private static final double simTurnInertia = 0.00001; // TODO
  private static final double simDriveInertia = 0.001; // TODO

  // Specific module constants:
  // Front Left
  private static final int frontLeftDriveMotorId = 0; // TODO
  private static final int frontLeftTurnMotorId = 0; // TODO
  private static final int frontLeftTurnEncoderId = 0; // TODO

  private static final double frontLeftXPos = 0; // TODO
  private static final double frontLeftYPos = 0; // TODO

  // Front Right
  private static final int frontRightDriveMotorId = 0; // TODO 
  private static final int frontRightTurnMotorId = 0; // TODO
  private static final int frontRightTurnEncoderId = 0; // TODO

  private static final double frontRightXPos = 0; // TODO
  private static final double frontRightYPos = 0; // TODO

  // Back Left
  private static final int backLeftDriveMotorId = 0; // TODO
  private static final int backLeftTurnMotorId = 0; // TODO
  private static final int backLeftTurnEncoderId = 0; // TODO

  private static final double backLeftXPos = 0; // TODO
  private static final double backLeftYPos = 0; // TODO

  // Back Right
  private static final int backRightDriveMotorId = 0; // TODO
  private static final int backRightTurnMotorId = 0; // TODO
  private static final int backRightTurnEncoderId = 0; // TODO

  private static final double backRightXPos = 0; // TODO
  private static final double backRightYPos = 0; // TODO

  // Auto-configured objects below
  public static final ElectricalLimitSettings driveElectricalLimitSettings =
      new ElectricalLimitSettings()
          .setBusCurrentLimit(driveSupplyCurrentLimit)
          .setBusCurrentLimitTime(driveSupplyCurrentTime)
          .setStatorCurrentLimit(driveStatorCurrentLimit);
  public static final ElectricalLimitSettings turnElectricalLimitSettings =
      new ElectricalLimitSettings()
          .setBusCurrentLimit(turnSupplyCurrentLimit)
          .setBusCurrentLimitTime(turnSupplyCurrentTime)
          .setStatorCurrentLimit(turnStatorCurrentLimit);

  private static final PIDSettings drivePIDSettings =
      new PIDSettings()
          .setPID(drivekP, 0, drivekD)
          .setStaticFeedforward(drivekS)
          .setVelocityFeedforward(drivekV);
  private static final PIDSettings turnPIDSettings =
      new PIDSettings()
          .setPID(turnkP, 0, turnkD)
          .setMotionProfileAccelLimit(turnAccelerationLimit)
          .setMotionProfileDeaccelLimit(turnDeaccelerationLimit)
          .setMotionProfileVelocityLimit(turnVelocityLimit);

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
