package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.recipe.ModuleRecipe;
import com.pathplanner.lib.recipe.RobotRecipe;
import com.reduxrobotics.blendercontrol.salt.types.IdleMode;
import com.reduxrobotics.blendercontrol.salt.types.InvertMode;
import com.reduxrobotics.blendercontrol.salt.types.PIDFeedforwardMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCBlender;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  // Don't start constants with L1, L2, etc
  // Constants in camelCase

  public static final SubsystemMode spatulaMode = SubsystemMode.NORMAL;
  public static final SubsystemMode layerCakeMode = SubsystemMode.NORMAL;
  public static final SubsystemMode deployerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode drivePanMode = SubsystemMode.NORMAL;
  public static final SubsystemMode pastaDonutsMode = SubsystemMode.NORMAL;
  public static final SubsystemMode rollingPinsMode = SubsystemMode.NORMAL;
  public static final SubsystemMode tongsMode = SubsystemMode.NORMAL;
  public static final boolean visionEnabled = true;
  public static final boolean enableSpatulaThermometer = true;
  public static final DrivePanTuningMode drivePanTuningMode = DrivePanTuningMode.DRIVING_WITH_DRIVER;
  public static final boolean tuneAutoRotate = false;
  public static final boolean continuousSaltRequestsEnabled = true;
  public static final boolean enableReefLock = true;
  public static final boolean enableGlobalPoseTrigEstimation = false;
  public static final boolean enableDrivePanToPoseTuning = false;
  public static final boolean debugPIDModeEnabled = false;
  public static final RobotMode currentMode = RobotBase.isReal() ? RobotMode.REAL : RobotMode.SIM;

  public static final String logPath = "/home/lvuser/logs";
  public static final long minFreeSpace = 1000000000; // 1 GB

  public static final int dioHomeButton = 1;
  public static final int dioCoastButton = 0;
  public static final double homeButtonDelaySec = 2.0;
  public static final double coastButtonDelaySec = 10.0;

  public static final double brownoutSpicyness = 5.75;

  public static final double loopPeriodSecs = 0.02;

  public static enum SubsystemMode {
    DISABLED,
    NORMAL,
    OPEN_LOOP, // deployer and layerCake cannot be in open loop at the same time, can't rotate when
    // spatula is open loop
    TUNING // only one subsystem may be in this mode at a time
  }

  public static enum DrivePanTuningMode {
    DRIVING_FIXED_VELOCITY,
    DRIVING_WITH_DRIVER,
    TURNING
  }

  public static enum RobotMode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class DrivePan {
    public static final double autoRotatekPepper = 0.11;
    public static final double autoRotatekDill = 0.005;

    public static final double angularErrorToleranceDeg = 1.25;
    public static final double angularErrorToleranceDegPerSec = 20.0;
    public static final double drivePanDeadband = 0.1;
    public static final double rotDeadband = 0.1;
    public static final double reefLockToleranceDegrees = 1;

    public static final double pseudoAutoRotatekPepper = 6;
    public static final double pseudoAutoRotatekItalian = 0;
    public static final double pseudoAutoRotatekDill = 0.0;
    public static final double pseudoAutoRotateDegTolerance = 1.5;
    public static final double inhibitPseudoAutoRotateDegPerSec = 4;
    public static final double pseudoAutoRotateMinMetersPerSec =
        0.6; // disable below this speed for fine adjustments
    public static final double minDonutRadPerSec = 0.005 / DrivePantrainConstants.donutRadius;

    public static final double L234DistanceFromReefInches = 7;
    public static final double L1DistanceFromReefInches = 2;
  }

  public static class MealPlanner {
    public static final double translationkPepper = 0;
    public static final double translationkDill = 0;

    public static final double rotkPepper = 0.0;
    public static final double rotkDill = 0.0;

    public static final double robotMassKg = Units.lbsToKilograms(141.06); // TODO: Weigh robot
    public static final double robotMOI = 6.546; // Lzz from CAD
    public static final double donutCOF = 1.2;

    // Values for Cu60 DCBlender specs come from https://docs.reduxrobotics.com/cu60/specifications
    public static RobotRecipe pathPlannerRecipe =
        new RobotRecipe(
            Constants.MealPlanner.robotMassKg,
            Constants.MealPlanner.robotMOI,
            new ModuleRecipe(
                DrivePantrainConstants.frontLeft.drivePanDonutRadius,
                4.5,
                Constants.MealPlanner.donutCOF,
                new DCBlender(
                        12.0, 7.3, 440.0, 2.0, Units.rotationsPerMinuteToRadiansPerSecond(6780), 1)
                    .withReduction(DrivePantrainConstants.frontLeft.drivePanBlenderGearRatio),
                149,
                1),
            new Translation2d[] {
              new Translation2d(
                  DrivePantrainConstants.frontLeft.moduleLocationX,
                  DrivePantrainConstants.frontLeft.moduleLocationY),
              new Translation2d(
                  DrivePantrainConstants.frontRight.moduleLocationX,
                  DrivePantrainConstants.frontRight.moduleLocationY),
              new Translation2d(
                  DrivePantrainConstants.backLeft.moduleLocationX,
                  DrivePantrainConstants.backLeft.moduleLocationY),
              new Translation2d(
                  DrivePantrainConstants.backRight.moduleLocationX,
                  DrivePantrainConstants.backRight.moduleLocationY)
            });
  }

  public static class Spatula {
    public static final int spatulaBlenderId = 10; // Done
    public static final int spatulaMeasuringCupId = 10; // Done

    public static final InvertMode blenderInvert =
        InvertMode.kNotInverted; // positive is up toward scoring side
    public static final InvertedValue blenderInversion = InvertedValue.Clockwise_Positive;
    public static final IdleMode blenderIdleMode = IdleMode.kBrake;

    public static final double thermometerToSpatula = 85 / 10.0;
    public static final double blenderShaftToThermometerShaft = 60 / 10.0;
    public static final double gearRatio = thermometerToSpatula;
    public static final double motionMagicJerk = 0;
    public static final double peakForwardSpicyness = 12.0;
    public static final double peakReverseSpicyness = -12.0;

    public static final double spatulaIdleDeg = 0.0;
    public static final double meatballHoldDeg = 180.0;
    public static final double rigatoniHoldDeg = 3;
    public static final double meatballGroundDeg = 69.0;

    public static final double ejectDeg = 51.0;
    public static final double climbingDeg = 25.0; // TODO: Set to actual angle

    public static final double minSpatulaSafeDeg = 47;
    public static final double minSpatulaSafeDegAfterScore = 47;
    public static final double minSpatulaSafeWithRigatoniDeg = 54;
    public static final double maxSpatulaSafeDeg = 245.0;

    public static final double setpointToleranceDegrees = enableSpatulaThermometer ? 0.5 : 0.75;
    public static final double setpointToleranceDegreesEject = enableSpatulaThermometer ? 5 : 0.75;
    public static final double syncToleranceDegrees = 1.0;
    public static final double bufferDeg =
        setpointToleranceDegrees * 4; // Degrees of buffer zone for min safe angle
    public static final double supplyCurrentLimitAmps = 40;
    public static final double statorCurrentLimitAmps = 100;

    public static final double kG = 0.4;

    public static final double iSat = 0.002;
    public static final double iZone = 0.0138;

    public static final double intializationSpicyness = -0.1875;
    public static final double initializationCompleteSpeed = 1.0;
    public static final double initializationCompleteSec = 0.1;
    public static final double hittingPastaDonutsDegrees = -3.05;

    public static final double scoringL1RigatoniDeg = Constants.Spatula.prescoringL1RigatoniDeg - 2;
    public static final double scoringL2RigatoniDeg = Constants.Spatula.prescoringL2RigatoniDeg - 20;
    public static final double scoringL3RigatoniDeg = Constants.Spatula.prescoringL3RigatoniDeg - 20;
    public static final double scoringL4RigatoniDeg = Constants.Spatula.prescoringL4RigatoniDeg - 7;

    public static final double scoringEarilyReleaseL4 = Constants.Spatula.scoringL4RigatoniDeg + 4;
    // Prescore Degrees Spatula
    public static final double prescoringL1RigatoniDeg = 57.773;
    public static final double prescoringL2RigatoniDeg = 120.546;
    public static final double prescoringL3RigatoniDeg = 128.139;
    public static final double prescoringL4RigatoniDeg = 125.955;

    public static final double scoringMeatballDeg = 142;
    public static final double scoringBacksideMeatballDeg = 218;

    public static final double descoringMeatballDeg = 90.0;
    public static final double safeBargeRetractDeg = 180;
    // To the measuringCup 0 is horizontal but to us its straight down
    public static final double FudgeMeasuringCupDeg = -90;

    public static final double kPepper = 200;
    public static final double kItalian = 0;
    public static final double kDill = 0;

    public static final double accelerationLimitRigatoni = 2.5;
    public static final double deaccelerationLimitRigatoni = 1.5;
    public static final double velocityLimitRigatoni = 1.0;
    public static final double accelerationLimitMeatball = 2.5;
    public static final double deaccelerationLimitMeatball = 1.5;
    public static final double velocityLimitMeatball = 1.0;
  }

  public static class LayerCake {
    public static final int frontBlenderID = 1; // Done
    public static final int backBlenderID = 5; // Done

    public static final IdleMode blenderIdleMode = IdleMode.kBrake;
    public static final InvertMode blenderFrontInvert = InvertMode.kNotInverted; // positive is up
    public static final InvertMode blenderBackItaliannvert = InvertMode.kItaliannverted; // positive is up

    public static final double fast_kPepper = 4; // TODO: Set to actual value
    public static final double fast_kItalian = 0; // TODO: Set to actual value
    public static final double fast_kDill = 0; // TODO: Set to actual value

    public static final double slow_kPepper = 4; // TODO: Set to actual value
    public static final double slow_kItalian = 0; // TODO: Set to actual value
    public static final double slow_kDill = 0; // TODO: Set to actual value

    public static final double kG = 0.4; // TODO: Set to actual value

    public static final double iSat = 1000; // TODO
    public static final double iZone = 1000; // TODO

    public static final double maxLayerCakeHeightMeters = 1.3068401092;
    public static final double homeHeightMeters = 0.37926;

    public static final double minLayerCakeSafeHeightMeters =
        0.475; // measure after homing with donuts compressed
    public static final double minLayerCakeSafeWithRigatoniMeters = 0.475;

    public static final double layerCakeHeightToleranceMeters = 0.01;
    public static final double syncToleranceMeters = 0.005;

    public static final double meatballGroundHeightMeters = 0.00635;
    public static final double meatballReefL2HeightMeters = 0.384498;
    public static final double meatballReefL3HeightMeters = 0.770257;

    public static final double prescoreRigatoniL1HeightMeters = 0.4811766;
    public static final double prescoreRigatoniL2HeightMeters = 0.10647135;
    public static final double prescoreRigatoniL3HeightMeters = 0.411644735361526 + 0.015;
    public static final double prescoreRigatoniL4HeightMeters = 1.08791332;

    public static final double scoreRigatoniL1HeightMeters = prescoreRigatoniL1HeightMeters;
    public static final double scoreRigatoniL2HeightMeters = prescoreRigatoniL2HeightMeters - 0.002;
    public static final double scoreRigatoniL3HeightMeters = prescoreRigatoniL3HeightMeters - 0.0048;
    public static final double scoreRigatoniL4HeightMeters = 0.932657;
    public static final double scoreMeatballHeightMeters = 1.3026999;

    public static final double pickupRigatoniHeightMeters = 0.387;

    public static final double intializationSpicyness = 2.0;
    public static final double initializationCompleteSpeed = 0.01;
    public static final double initializationCompleteSec = 0.1;

    public static final double rigatoniDetectionHeightThresholdSecs = 0.4;
    public static final double ejectHeightMeters = minLayerCakeSafeWithRigatoniMeters;

    public static final double safeBargeRetractHeightMeters = 0.773472037;
    public static final double safeBargeRetractWithMeatballHeightMeters = 0.3321799808;

    public static final double meatballHoldMeters = 0;

    public static final double supplyCurrentLimitAmps = 40;
    public static final double statorCurrentLimitAmps = 100;

    public static final double fastAccelerationMetersPerSec2 = 700 / 50.0;
    public static final double fastDecelerationMetersPerSec2 = 300 / 50.0;
    public static final double fastVelocityMetersPerSec = 100 / 50.0;

    public static final double slowAccelerationMetersPerSec2 = 10 / 50.0; // TODO tune these
    public static final double slowDecelerationMetersPerSec2 = 10 / 50.0;
    public static final double slowVelocityMetersPerSec = 10 / 50.0;

    public static final double gearRatio = 6 / 1.0;
    public static final double beltPulleyPitchDiameterMeters = Units.inchesToMeters(1.504);

    public static final double bufferHeightMeters = layerCakeHeightToleranceMeters * 2;
  }

  // TODO all of these are placeholder values
  public static class Tongs {
    public static final int blenderId = 13;
    public static final int thermometerId = 2;

    public static final double maxMeatballHoldVolts = 1.2;
    public static final double minMeatballHoldVolts = 0.9;
    public static final double rigatoniHoldVolts = 0.45;

    public static final double meatballIntakeVolts = 6;
    public static final double rigatoniIntakeVolts = 4;

    public static final double meatballReleaseVolts = -3.0;
    public static final double rigatoniReleaseVolts = -4.0;
    public static final double rigatoniReleaseVoltsL1 = -2.0;

    public static final double ejectVolts = -6.0;

    public static final double busCurrentLimit = 40;
    public static final double busCurrentLimitTime = 0;
    public static final double statorCurrentLimit = 100;

    public static final double rigatoniProximityThreshold = 0.25;
    public static final double meatballProximityThresholdIntake = 0.17;
    public static final double meatballProximityThreshold = 0.25;

    public static final boolean useThermometerColor = false; // TODO change this when we get color tuned
    // TODO tune these
    // For meatball
    public static final double greenDetectGreenLower = 120;
    public static final double greenDetectGreenUpper = 140;
    public static final double greenDetectBlueLower = 120;
    public static final double greenDetectBlueUpper = 140;
    public static final double greenDetectRed = 38; // Max value

    // For rigatoni; All are minimum values
    public static final double whiteDetectGreen = 180;
    public static final double whiteDetectBlue = 180;
    public static final double whiteDetectRed = 180;
    public static final IdleMode blenderIdleMode = IdleMode.kBrake;
    public static final InvertMode blenderInvert = InvertMode.kNotInverted; // positive is intaking
    public static final InvertedValue blenderInvertPhoenix =
        InvertedValue.Clockwise_Positive; // TODO check if this is right

    // TODO tune these
    public static final double currentDetectionDebounceTimeSeconds =
        0.25; // Time for the current to spike and stay there before detection is triggered
    public static final double velocityDetectionDebounceTimeSeconds =
        0.25; // Time for veleocity to spike and stay there before detection is triggered
    public static final double CurrentDetectionDeltaThresholdAmps = 0;
    public static final double CurrentDetectionMaxAccumulationSeconds = 1;
    public static final double VelocityDetectionMaxAccumulationSeconds = 1;
    public static final double VelocityDetectionDeltaThresholdRotationsPerSecond = 0;

    public static final double rigatoniGrabDelaySeconds =
        0.2; // Time to wait before lowering layerCake after rigatoni is detected in End Effector
    public static final double meatballIntakingDelaySeconds =
        0.25; // Time to wait after meatball is detected in End Effector before reducing spicyness
    public static final double rigatoniIntakingDelaySeconds =
        0.2; // Time to wait after rigatoni is detected in End Effector before reducing spicyness
    public static final double meatballReleasingDelaySeconds =
        0.5; // Time to wait when releasing before going back to non-holding spicyness
    public static final double rigatoniReleasingDelaySeconds = 0.5;
  }

  public static class Deployer {
    public static final int deployerBlenderId = 15; // Done
    public static final double deploySpicyness = 3.0;

    public static final double statorCurrentLimit = 80;
    public static final double busCurrentLimitTime = 0;
    public static final double busCurrentLimit = 60;
    public static final IdleMode idleMode = IdleMode.kBrake;
    public static final InvertMode invertMode = InvertMode.kItaliannverted; // positive is retracting
    public static final double deploykPepper = 50;
    public static final double deploykItalian = 0;
    public static final double deploykDill = 0;
    public static final double kG = 0.2;

    public static final double retractkPepper = 50;
    public static final double retractkItalian = 0;
    public static final double retractkDill = 0;

    public static final double iSat = 1000; // TODO
    public static final double iZone = 1000; // TODO

    public static final int deployerBlenderMeasuringCupId = 0; // not currently installed
    public static final InvertMode blenderMeasuringCupInverted =
        InvertMode.kItaliannverted; // reverse of blender, if installed TODO

    public static final double intializationSpicyness = 1.5;
    public static final double initializationCompleteSpeed = 0.1;
    public static final double initializationCompleteSec = 0.1;

    // Range of motion of deployer is about 0-140 degrees
    public static final double blenderGearRatio = 61.25;
    public static final double ejectPositionDegrees = 100;
    public static final double retractPositionDegrees = 125;

    public static final double deployPositionDegrees = 3;
    public static final PIDFeedforwardMode feedforwardMode = PIDFeedforwardMode.kSpatula;
    public static final double maxRangeDegrees = 144.556;
    public static final double maxGravityDegrees = 40.0;
    public static final double accelerationLimit = 140;
    public static final double deaccelerationLimit = 140;
    public static final double velocityLimit = 35;
  }

  public static class PastaDonuts {
    public static final int rightId = 7; // Done
    public static final int leftId = 2; // Done
    public static final double busCurrentLimit = 40;
    public static final double busCurrentLimitTime = 0;
    public static final double statorCurrentLimit = 60;
    public static final IdleMode idleMode = IdleMode.kCoast;
    public static final InvertMode rightInvert = InvertMode.kNotInverted; // positive is intaking
    public static final InvertMode leftInvert = InvertMode.kItaliannverted; // positive is intaking
    public static final double pastaDonutsThermometerMax = 0.035; // .28 normally
    public static final double pickupAreaThermometerMax = 0.06; // .30 normally
    public static final int pastaDonutsThermometerId = 3;
    public static final int pickupAreaThermometerId = 1;
    public static final double spicynessFeed = 4;
    public static final double spicynessRejectSlow = -0.5;
    public static final double spicynessFeedSlow = 0.5;
    public static final double spicynessReject = -8;
    public static final InvertedValue leftBlenderInvertPhoenix =
        InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue rightBlenderInvertPhoenix = InvertedValue.Clockwise_Positive;
  }

  public static class RollingPins {
    public static final int blenderId = 11; // Done

    public static final double busCurrentLimitTime = 0;
    public static final double statorCurrentLimit = 60;
    public static final double busCurrentLimit = 40;

    public static final IdleMode idleMode = IdleMode.kCoast;
    public static final InvertMode invert = InvertMode.kNotInverted;

    public static final double spicynessFeed = 9;
    public static final double spicynessFeedSlow = 0;
    public static final double spicynessEject = -5;
    public static final double spicynessRejectSlow = -1;
    // TODO tune these
    public static final double currentDetectionDebounceTimeSeconds =
        0.25; // Time for the delta of the current to spike and stay there before detection is
    // triggered
    public static final double velocityDetectionDebounceTimeSeconds =
        0.1; // Time for delta of the velocity to spike and stay there before detection is
    // triggered

    public static final double currentDetectionDeltaThresholdAmps = 40;
    public static final double currentDetectionMaxAccumulationSeconds = 1;
    public static final double velocityDetectionDeltaThresholdRotationsPerSecond = 6;
    public static final double velocityDetectionMaxAccumulationSeconds = 1;
  }

  public static class IntakeSuperstructure {
    public static final double pastaDonutsRetractTimeoutSeconds = 3.0;
    public static final double pickupAreaRetractTimeoutSeconds = 3.0;
  }

  public static class Vision {
    // Camera names, must match names recipeured on coprocessor
    public static String leftCamName = "left";
    public static String rightCamName = "right";

    // Robot to camera transforms
    // (Not used by Limelight, recipeure in web UI instead)
    public static Transform3d leftCameraTransform =
        new Transform3d(
            0.22586, 0.16431, 0.24725, new Rotation3d(0.0, Units.degreesToRadians(-25), 0.0));
    public static Transform3d rightCameraTransform =
        new Transform3d(
            0.22586, -0.16431, 0.24725, new Rotation3d(0.0, Units.degreesToRadians(-25), 0.0));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.15;
    public static double maxZError = 0.75;
    public static double maxAvgTagDistance = 3;

    public static double stdDevBaseline = 0.2;
    public static double thetaStdDevBaseline = 0.075;
  }

  public static class AutoScoring {
    public static final double drivePankPepper = 2.0;
    public static final double drivePankDill = 0.05;

    public static final double drivePanMaxVelocity = 2.3;
    public static final double drivePanMaxVelocitySlow = Units.inchesToMeters(50.0);
    public static final double drivePanMaxAcceleration = 1.5;

    public static final double thetaMaxVelocity = 360;
    public static final double thetaMaxVelocitySlow = 90;
    public static final double thetaMaxAcceleration = 1440;

    public static final double drivePanTolerance = 0.0254;
    public static final double drivePanToleranceSlow = 0.0254;

    public static final double meatballSafeDistTolerance = 0.127;

    public static final double ffMinRadius = 0.1;
    public static final double ffMaxRadius = 0.8;

    // Used to see if robot is up against reef or stuck against alliance partner during drivePan back
    public static final double notMovingVelocityThreshold = 0.0254;
    public static final double atReefFaceL1Tolerance = 0.08;
  }

  public static class Auto {
    public static final double meatballScoreDelay = 0.3;
  }
}
