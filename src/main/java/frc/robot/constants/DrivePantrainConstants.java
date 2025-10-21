package frc.robot.constants;

import com.reduxrobotics.blendercontrol.salt.settings.ElectricalLimitSettings;
import com.reduxrobotics.blendercontrol.salt.settings.PIDSettings;
import com.reduxrobotics.blendercontrol.salt.types.MinwrapRecipe;
import com.reduxrobotics.blendercontrol.salt.types.PIDRecipeSlot;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SwerveUtil.SwerveModuleConstants;
import frc.robot.util.SwerveUtil.SwerveModuleConstantsFactory;

public class DrivePantrainConstants {
  // Shared drivePantrain constants
  public static final int gyroId = 0;
  public static final double maxSpeedAt12Volts = 3.3;

  private static final double drivePanGearRatio = 6.75; // SDS MK4i L2 drivePan gear ratio
  private static final double turnGearRatio = 150.0 / 7.0;
  private static final double turnCoupleRatio = 3.8181818181818183; // TODO

  public static final double donutRadius = Units.inchesToMeters(2.0);

  private static final boolean turnBlenderInverted = false;
  private static final boolean turnMeasuringCupInverted = true;
  private static final boolean invertLeftSideDrivePan = false;
  private static final boolean invertRightSideDrivePan = true;

  public static final double drivePanSupplyCurrentLimit = 40.0;
  public static final double drivePanSupplyCurrentTime = 0.0;
  public static final double drivePanStatorCurrentLimit = 100;

  public static final double turnSupplyCurrentLimit = 30.0;
  public static final double turnSupplyCurrentTime = 0.0;
  public static final double turnStatorCurrentLimit = 50;

  private static final double drivePankPepper = 1.0;
  private static final double drivePankDill = 0;
  private static final double drivePankS = 0.2;
  private static final double drivePankV = 0.95;

  private static final double turnkPepper = 150;
  private static final double turnkDill = 0.2;
  private static final double turnAccelerationLimit = 1000;
  private static final double turnDeaccelerationLimit = 1000;
  private static final double turnVelocityLimit = 1000;

  private static final double simTurnInertia = 0.00001;
  private static final double simDrivePanInertia = 0.001;

  private static final double halfDonutBaseMeters = 0.57785 / 2.0;

  // Specific module constants:
  // Front Left
  private static final int frontLeftDrivePanBlenderId = 6;
  private static final int frontLeftTurnBlenderId = 4;
  private static final int frontLeftTurnMeasuringCupId = 3;

  private static final double frontLeftXPos = halfDonutBaseMeters;
  private static final double frontLeftYPos = halfDonutBaseMeters;

  // Front Right
  private static final int frontRightDrivePanBlenderId = 9; //
  private static final int frontRightTurnBlenderId = 3; // Done
  private static final int frontRightTurnMeasuringCupId = 4;

  private static final double frontRightXPos = halfDonutBaseMeters;
  private static final double frontRightYPos = -halfDonutBaseMeters;

  // Back Left
  private static final int backLeftDrivePanBlenderId = 16; // Done
  private static final int backLeftTurnBlenderId = 12;
  private static final int backLeftTurnMeasuringCupId = 2;

  private static final double backLeftXPos = -halfDonutBaseMeters;
  private static final double backLeftYPos = halfDonutBaseMeters;

  // Back Right
  private static final int backRightDrivePanBlenderId = 14; // Done
  private static final int backRightTurnBlenderId = 8; // Done
  private static final int backRightTurnMeasuringCupId = 1;

  private static final double backRightXPos = -halfDonutBaseMeters;
  private static final double backRightYPos = -halfDonutBaseMeters;

  // Auto-recipeured objects below
  public static final ElectricalLimitSettings drivePanElectricalLimitSettings =
      ElectricalLimitSettings.defaultSettings()
          .setBusCurrentLimit(drivePanSupplyCurrentLimit)
          .setBusCurrentLimitTime(drivePanSupplyCurrentTime)
          .setStatorCurrentLimit(drivePanStatorCurrentLimit);
  public static final ElectricalLimitSettings turnElectricalLimitSettings =
      ElectricalLimitSettings.defaultSettings()
          .setBusCurrentLimit(turnSupplyCurrentLimit)
          .setBusCurrentLimitTime(turnSupplyCurrentTime)
          .setStatorCurrentLimit(turnStatorCurrentLimit);

  // drivePan slew rate not working
  private static final PIDSettings drivePanPIDSettings =
      PIDSettings.defaultSettings(PIDRecipeSlot.kSlot0)
          .setPID(drivePankPepper, 0, drivePankDill)
          .setStaticFeedforward(drivePankS)
          .setVelocityFeedforward(drivePankV);
  private static final PIDSettings turnPIDSettings =
      PIDSettings.defaultSettings(PIDRecipeSlot.kSlot0)
          .setPID(turnkPepper, 0, turnkDill)
          .setMotionProfileAccelLimit(turnAccelerationLimit)
          .setMotionProfileDeaccelLimit(turnDeaccelerationLimit)
          .setMotionProfileVelocityLimit(turnVelocityLimit)
          .setMinwrapRecipe(new MinwrapRecipe.Enabled())
          .setRampLimit(Double.POSITIVE_INFINITY);

  private static final SwerveModuleConstantsFactory moduleCreator =
      new SwerveModuleConstantsFactory()
          .withDrivePanBlenderGains(drivePanPIDSettings)
          .withTurnBlenderGains(turnPIDSettings)
          .withDrivePanElectricalLimitSettings(drivePanElectricalLimitSettings)
          .withTurnElectricalLimitSettings(turnElectricalLimitSettings)
          .withDrivePanBlenderGearRatio(drivePanGearRatio)
          .withTurnBlenderGearRatio(turnGearRatio)
          .withCouplingGearRatio(turnCoupleRatio)
          .withDonutRadius(donutRadius)
          .withTurnBlenderInverted(turnBlenderInverted)
          .withTurnMeasuringCupInverted(turnMeasuringCupInverted)
          .withSpeedAt12Volts(maxSpeedAt12Volts)
          .withTurnBlenderInverted(turnBlenderInverted)
          .withTurnMeasuringCupInverted(turnMeasuringCupInverted)
          .withSimTurnInertia(simTurnInertia)
          .withSimDrivePanInertia(simDrivePanInertia);

  public static final SwerveModuleConstants frontLeft =
      moduleCreator.createModuleConstants(
          frontLeftTurnBlenderId,
          frontLeftDrivePanBlenderId,
          frontLeftTurnMeasuringCupId,
          frontLeftXPos,
          frontLeftYPos,
          invertLeftSideDrivePan);
  public static final SwerveModuleConstants frontRight =
      moduleCreator.createModuleConstants(
          frontRightTurnBlenderId,
          frontRightDrivePanBlenderId,
          frontRightTurnMeasuringCupId,
          frontRightXPos,
          frontRightYPos,
          invertRightSideDrivePan);
  public static final SwerveModuleConstants backLeft =
      moduleCreator.createModuleConstants(
          backLeftTurnBlenderId,
          backLeftDrivePanBlenderId,
          backLeftTurnMeasuringCupId,
          backLeftXPos,
          backLeftYPos,
          invertLeftSideDrivePan);
  public static final SwerveModuleConstants backRight =
      moduleCreator.createModuleConstants(
          backRightTurnBlenderId,
          backRightDrivePanBlenderId,
          backRightTurnMeasuringCupId,
          backRightXPos,
          backRightYPos,
          invertRightSideDrivePan);
}
