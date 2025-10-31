package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.InvertMode;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDFeedforwardMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Trigon.simulatedfield.SimulatedGamePieceConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  // Don't start constants with L1, L2, etc
  // Constants in camelCase

  public static final SubsystemMode armMode = SubsystemMode.NORMAL;
  public static final SubsystemMode elevatorMode = SubsystemMode.NORMAL;
  public static final SubsystemMode deployerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode driveMode = SubsystemMode.NORMAL;
  public static final SubsystemMode indexerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode rollersMode = SubsystemMode.NORMAL;
  public static final SubsystemMode endEffectorMode = SubsystemMode.NORMAL;
  public static final boolean visionEnabled = true;
  public static final boolean visionObjectDetectionEnabled = true;

  public static final DriveTuningMode driveTuningMode = DriveTuningMode.DRIVE_AUTO;
  public static final boolean tuneAutoRotate = false;
  public static final boolean continuousNitrateRequestsEnabled = true;
  public static final boolean enableReefLock = true;
  public static final boolean enableGlobalPoseTrigEstimation = false;
  public static final boolean enableDriveToPoseTuning = false;
  public static final boolean debugPIDModeEnabled = false;
  public static final boolean wantDriveTestAutos = true;
  public static final boolean enableObjectDetectionDebug = true;

  public static final boolean buzz = false;

  public static final RobotMode currentMode = RobotBase.isReal() ? RobotMode.REAL : RobotMode.SIM;

  public static final String logPath = "/home/lvuser/logs";
  public static final long minFreeSpace = 1000000000; // 1 GB

  public static final int dioHomeButton = 1;
  public static final int dioCoastButton = 0;
  public static final double homeButtonDelaySec = 2.0;
  public static final double coastButtonDelaySec = 10.0;

  public static final double brownoutVoltage = 5.75;

  public static final double loopPeriodSecs = 0.02;

  public static enum SubsystemMode {
    DISABLED,
    NORMAL,
    OPEN_LOOP, // deployer and elevator cannot be in open loop at the same time, can't rotate when
    // arm is open loop
    TUNING // only one subsystem may be in this mode at a time
  }

  public static enum DriveTuningMode {
    DRIVING_FIXED_VELOCITY,
    DRIVE_AUTO,
    DRIVE_MANUAL,
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

  public static class Drive {
    public static final double autoRotatekP = 0.11;
    public static final double autoRotatekD = 0.005;

    public static final double algaeSafeRetractDistanceXPlaceBlue = 7.5;
    public static final double algaeSafeRetractDistanceXPlaceRed = 10.261;

    public static final double angularErrorToleranceDeg = 1.25;
    public static final double angularErrorToleranceDegPerSec = 20.0;
    public static final double driveDeadband = 0.1;
    public static final double rotDeadband = 0.1;
    public static final double reefLockToleranceDegrees = 1;

    public static final double pseudoAutoRotatekP = 6;
    public static final double pseudoAutoRotatekI = 0;
    public static final double pseudoAutoRotatekD = 0.0;
    public static final double pseudoAutoRotateDegTolerance = 1.5;
    public static final double inhibitPseudoAutoRotateDegPerSec = 4;
    public static final double pseudoAutoRotateMinMetersPerSec =
        0.6; // disable below this speed for fine adjustments
    public static final double minWheelRadPerSec = 0.005 / DrivetrainConstants.wheelRadius;

    public static final double L234DistanceFromReefInches = 7;
    public static final double L1DistanceFromReefInches = 2;

    // to adjust for slippage cause by uneven weight distribution
    public static final double[] modulePositionScaling =
        new double[] {0.856, 0.941, 1, 0.912}; // module 0 slips
  }

  public static class PathPlanner {
    public static final double translationkP = 6;
    public static final double translationkD = 0;

    public static final double rotkP = 1;
    public static final double rotkD = 0.0;

    public static final double robotMassKg = Units.lbsToKilograms(141.06); // TODO: Weigh robot
    public static final double robotMOI = 6.546; // Lzz from CAD
    public static final double wheelCOF = 1.2;

    // Values for Cu60 DCMotor specs come from https://docs.reduxrobotics.com/cu60/specifications
    public static RobotConfig pathPlannerConfig =
        new RobotConfig(
            Constants.PathPlanner.robotMassKg,
            Constants.PathPlanner.robotMOI,
            new ModuleConfig(
                DrivetrainConstants.frontLeft.driveWheelRadius,
                4.5,
                Constants.PathPlanner.wheelCOF,
                new DCMotor(
                        12.0, 7.3, 440.0, 2.0, Units.rotationsPerMinuteToRadiansPerSecond(6780), 1)
                    .withReduction(DrivetrainConstants.frontLeft.driveMotorGearRatio),
                149,
                1),
            new Translation2d[] {
              new Translation2d(
                  DrivetrainConstants.frontLeft.moduleLocationX,
                  DrivetrainConstants.frontLeft.moduleLocationY),
              new Translation2d(
                  DrivetrainConstants.frontRight.moduleLocationX,
                  DrivetrainConstants.frontRight.moduleLocationY),
              new Translation2d(
                  DrivetrainConstants.backLeft.moduleLocationX,
                  DrivetrainConstants.backLeft.moduleLocationY),
              new Translation2d(
                  DrivetrainConstants.backRight.moduleLocationX,
                  DrivetrainConstants.backRight.moduleLocationY)
            });
  }

  public static class Arm {
    public static final int armMotorId = 19;

    public static final InvertMode motorInvert =
        InvertMode.kNotInverted; // positive is up toward scoring side
    public static final InvertedValue motorInversion = InvertedValue.Clockwise_Positive;
    public static final IdleMode motorIdleMode = IdleMode.kBrake;

    public static final double sensorToArm = 85 / 10.0;
    public static final double motorShaftToSensorShaft = 60 / 10.0;
    public static final double gearRatio = sensorToArm * motorShaftToSensorShaft;
    public static final double motionMagicJerk = 0;
    public static final double peakForwardVoltage = 12.0;
    public static final double peakReverseVoltage = -12.0;

    public static final double armIdleDeg = 0.0;
    public static final double algaeHoldDeg = 180.0;
    public static final double coralHoldDeg = 3;
    public static final double algaeGroundDeg = 69.0;

    public static final double ejectDeg = 51.0;
    public static final double climbingDeg = 25.0;

    public static final double minArmSafeDeg = 47;
    public static final double minArmSafeDegAfterScore = 47;
    public static final double minArmSafeWithCoralDeg = 54;
    public static final double maxArmSafeDeg = 245.0;
    public static final double maxArmDegToLowerElevator = 185.0;

    public static final double setpointToleranceDegrees = 0.7;
    public static final double setpointToleranceDegreesEject = 5.0;
    public static final double syncToleranceDegrees = 1.0;
    public static final double bufferDeg =
        setpointToleranceDegrees * 4; // Degrees of buffer zone for min safe angle
    public static final double supplyCurrentLimitAmps = 40;
    public static final double statorCurrentLimitAmps = 100;

    public static final double kGCoral = 0.33;
    public static final double kGAlgae = 0.5;

    public static final double intializationVoltage = -0.1875;
    public static final double initializationCompleteSpeed = 1.0;
    public static final double initializationCompleteSec = 0.1;
    public static final double hittingIndexerDegrees = -3.05;

    public static final double scoringL1CoralDeg = Constants.Arm.prescoringL1CoralDeg - 2;
    public static final double scoringL2CoralDeg = Constants.Arm.prescoringL2CoralDeg - 20;
    public static final double scoringL3CoralDeg = Constants.Arm.prescoringL3CoralDeg - 20;
    public static final double scoringL4CoralDeg = Constants.Arm.prescoringL4CoralDeg - 7;

    public static final double scoringEarilyReleaseL4 = Constants.Arm.scoringL4CoralDeg + 4;
    // Prescore Degrees Arm
    public static final double prescoringL1CoralDeg = 57.773;
    public static final double prescoringL2CoralDeg = 120.546;
    public static final double prescoringL3CoralDeg = 128.139;
    public static final double prescoringL4CoralDeg = 125.955;

    public static final double scoringAlgaeDeg = 142;
    public static final double scoringBacksideAlgaeDeg = 218;

    public static final double descoringAlgaeDeg = 90.0;
    public static final double safeBargeRetractDeg = 180;
    // To the encoder 0 is horizontal but to us its straight down
    public static final double OffsetEncoderDeg = -90;

    public static final double kP = 200;
    public static final double kI = 0;
    public static final double kDCoral = 4;
    public static final double kDAlgae = 18;

    public static final double accelerationLimit = 8.0;
    public static final double velocityLimit = 3.0;
  }

  public static class Elevator {
    public static final int frontMotorID = 21;
    public static final int backMotorID = 15;

    public static final IdleMode motorIdleMode = IdleMode.kBrake;
    public static final InvertMode motorFrontInvert = InvertMode.kNotInverted; // positive is up
    public static final InvertMode motorBackInvert = InvertMode.kInverted; // positive is up
    public static final InvertedValue leaderInversion = InvertedValue.Clockwise_Positive;
    public static final InvertedValue followerInversion = InvertedValue.CounterClockwise_Positive;

    public static final double fast_kP = 4;
    public static final double fast_kI = 0;
    public static final double fast_kD = 0;

    public static final double slow_kP = 4;
    public static final double slow_kI = 0;
    public static final double slow_kD = 0;

    public static final double kG = 0.35;

    public static final double maxElevatorHeightMeters = 1.3068401092;
    public static final double homeHeightMeters = 0.4826474; // on wood (on craddle = 0.37926)

    public static final double minElevatorSafeHeightMeters =
        0.475; // measure after homing with wheels compressed
    public static final double minElevatorSafeWithCoralMeters = 0.475;

    public static final double elevatorHeightToleranceMeters = 0.01;
    public static final double syncToleranceMeters = 0.005;

    public static final double algaeGroundHeightMeters = 0.00635;
    public static final double algaeReefL2HeightMeters = 0.384498;
    public static final double algaeReefL3HeightMeters = 0.770257;

    public static final double prescoreCoralL1HeightMeters = 0.4811766;
    public static final double prescoreCoralL2HeightMeters = 0.10647135;
    public static final double prescoreCoralL3HeightMeters = 0.411644735361526 + 0.015;
    public static final double prescoreCoralL4HeightMeters = 1.08791332;

    public static final double scoreCoralL1HeightMeters = prescoreCoralL1HeightMeters;
    public static final double scoreCoralL2HeightMeters = prescoreCoralL2HeightMeters - 0.002;
    public static final double scoreCoralL3HeightMeters = prescoreCoralL3HeightMeters - 0.0048;
    public static final double scoreCoralL4HeightMeters = 0.932657;
    public static final double scoreAlgaeHeightMeters = 1.3026999;

    public static final double pickupCoralHeightMeters = 0.387;

    public static final double intializationVoltage = 2.0;
    public static final double initializationCompleteSpeed = 0.01;
    public static final double initializationCompleteSec = 0.1;

    public static final double coralDetectionHeightThresholdSecs = 0.4;
    public static final double ejectHeightMeters = minElevatorSafeWithCoralMeters;

    public static final double safeBargeRetractHeightMeters = 0.773472037;
    public static final double safeBargeRetractWithAlgaeHeightMeters = 0.3321799808;

    public static final double algaeHoldMeters = 0;

    public static final double supplyCurrentLimitAmps = 40;
    public static final double statorCurrentLimitAmps = 100;

    public static final double fastAccelerationMetersPerSec2 = 700 / 50.0;
    public static final double fastDecelerationMetersPerSec2 = 300 / 50.0;
    public static final double fastVelocityMetersPerSec = 100 / 50.0;

    public static final double motionMagicJerk = 0;

    public static final double slowAccelerationMetersPerSec2 = 10 / 50.0; // TODO tune these
    public static final double slowDecelerationMetersPerSec2 = 10 / 50.0;
    public static final double slowVelocityMetersPerSec = 10 / 50.0;

    public static final double gearRatio = 6 / 1.0;
    public static final double beltPulleyPitchDiameterMeters = Units.inchesToMeters(1.504);

    public static final double bufferHeightMeters = elevatorHeightToleranceMeters * 2;
  }

  public static class EndEffector {
    public static final int motorId = 22;
    public static final int sensorId = 2;

    public static final double maxAlgaeHoldVolts = 1.0;
    public static final double minAlgaeHoldVolts = 0.9;
    public static final double coralHoldVolts = 0.45;

    public static final double algaeIntakeVolts = 6;
    public static final double coralIntakeVolts = 4;

    public static final double algaeReleaseVolts = -6.0;
    public static final double coralReleaseVolts = -4.0;
    public static final double coralReleaseVoltsL1 = -2.0;

    public static final double ejectVolts = -6.0;

    public static final double busCurrentLimit = 40;
    public static final double busCurrentLimitTime = 0;
    public static final double statorCurrentLimit = 100;

    public static final double coralProximityThreshold = 0.25;
    public static final double algaeProximityThresholdIntake = 0.17;
    public static final double algaeProximityThreshold = 0.25;

    public static final boolean useSensorColor = false; // TODO change this when we get color tuned
    // TODO tune these
    // For algae
    public static final double greenDetectGreenLower = 120;
    public static final double greenDetectGreenUpper = 140;
    public static final double greenDetectBlueLower = 120;
    public static final double greenDetectBlueUpper = 140;
    public static final double greenDetectRed = 38; // Max value

    // For coral; All are minimum values
    public static final double whiteDetectGreen = 180;
    public static final double whiteDetectBlue = 180;
    public static final double whiteDetectRed = 180;
    public static final IdleMode motorIdleMode = IdleMode.kBrake;
    public static final InvertMode motorInvert = InvertMode.kNotInverted; // positive is intaking
    public static final InvertedValue motorInvertPhoenix = InvertedValue.Clockwise_Positive;

    // TODO tune these
    public static final double currentDetectionDebounceTimeSeconds =
        0.25; // Time for the current to spike and stay there before detection is triggered
    public static final double velocityDetectionDebounceTimeSeconds =
        0.25; // Time for veleocity to spike and stay there before detection is triggered
    public static final double CurrentDetectionDeltaThresholdAmps = 0;
    public static final double CurrentDetectionMaxAccumulationSeconds = 1;
    public static final double VelocityDetectionMaxAccumulationSeconds = 1;
    public static final double VelocityDetectionDeltaThresholdRotationsPerSecond = 0;

    public static final double coralGrabDelaySeconds =
        0.2; // Time to wait before lowering elevator after coral is detected in End Effector
    public static final double algaeIntakingDelaySeconds =
        0.25; // Time to wait after algae is detected in End Effector before reducing voltage
    public static final double coralIntakingDelaySeconds =
        0.2; // Time to wait after coral is detected in End Effector before reducing voltage
    public static final double algaeReleasingDelaySeconds =
        0.5; // Time to wait when releasing before going back to non-holding voltage
    public static final double coralReleasingDelaySeconds = 0.5;
  }

  public static class Deployer {
    public static final int deployerMotorId = 15;
    public static final double deployVoltage = 3.0;

    public static final double statorCurrentLimit = 80;
    public static final double busCurrentLimitTime = 0;
    public static final double busCurrentLimit = 60;
    public static final IdleMode idleMode = IdleMode.kBrake;
    public static final InvertMode invertMode = InvertMode.kInverted; // positive is retracting
    public static final double deploykP = 50;
    public static final double deploykI = 0;
    public static final double deploykD = 0;
    public static final double kG = 0.2;

    public static final double retractkP = 50;
    public static final double retractkI = 0;
    public static final double retractkD = 0;

    public static final double iSat = 1000;
    public static final double iZone = 1000;

    public static final int deployerMotorEncoderId = 0; // not currently installed
    public static final InvertMode motorEncoderInverted =
        InvertMode.kInverted; // reverse of motor, if installed TODO

    public static final double intializationVoltage = 1.5;
    public static final double initializationCompleteSpeed = 0.1;
    public static final double initializationCompleteSec = 0.1;

    // Range of motion of deployer is about 0-140 degrees
    public static final double motorGearRatio = 61.25;
    public static final double ejectPositionDegrees = 100;
    public static final double retractPositionDegrees = 125;

    public static final double deployPositionDegrees = 3;
    public static final PIDFeedforwardMode feedforwardMode = PIDFeedforwardMode.kArm;
    public static final double maxRangeDegrees = 144.556;
    public static final double maxGravityDegrees = 40.0;
    public static final double accelerationLimit = 140;
    public static final double deaccelerationLimit = 140;
    public static final double velocityLimit = 35;
  }

  public static class Indexer {
    public static final int rightId = 5; // Done
    public static final int leftId = 2; // Done
    public static final double busCurrentLimit = 40;
    public static final double busCurrentLimitTime = 0;
    public static final double statorCurrentLimit = 60;
    public static final IdleMode idleMode = IdleMode.kCoast;
    public static final InvertMode rightInvert = InvertMode.kNotInverted; // positive is intaking
    public static final InvertMode leftInvert = InvertMode.kInverted; // positive is intaking
    public static final double indexerSensorMax = 0.035; // .28 normally
    public static final double pickupAreaSensorMax = 0.06; // .30 normally
    public static final int indexerSensorId = 3;
    public static final int pickupAreaSensorId = 1;
    public static final double voltageFeed = 4;
    public static final double voltageRejectSlow = -0.5;
    public static final double voltageFeedSlow = 0.5;
    public static final double voltageReject = -8;
    public static final InvertedValue rightMotorInvertPhoenix =
        InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue leftMotorInvertPhoenix = InvertedValue.Clockwise_Positive;
  }

  public static class Rollers {
    public static final int motorId = 20;

    public static final double busCurrentLimitTime = 0;
    public static final double statorCurrentLimit = 60;
    public static final double busCurrentLimit = 40;

    public static final IdleMode idleMode = IdleMode.kCoast;
    public static final InvertMode invert = InvertMode.kNotInverted;
    public static final InvertedValue motorInvertPhoenix = InvertedValue.Clockwise_Positive;

    public static final double voltageFeed = 9;
    public static final double voltageFeedSlow = 0;
    public static final double voltageEject = -5;
    public static final double voltageRejectSlow = -1;
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
    public static final double indexerRetractTimeoutSeconds = 3.0;
    public static final double pickupAreaRetractTimeoutSeconds = 3.0;
  }

  public static class Vision {
    // Camera names, must match names configured on coprocessor
    public static String leftCamName = "left";
    public static String rightCamName = "right";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
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
    public static final double drivekP = 2;
    public static final double drivekD = 0;

    public static final double driveMaxVelocity = 2.3;
    public static final double driveMaxVelocitySlow = 1.27;
    public static final double driveMaxAcceleration = 4;
    public static final double driveMaxDeceleration = 1.2;
    public static final double driveMaxDecelerationHigh = 0.7;

    public static final double thetaMaxVelocity = 360;
    public static final double thetaMaxVelocitySlow = 90;
    public static final double thetaMaxAcceleration = 1440;

    public static final double driveTolerance = 0.0254;
    public static final double driveToleranceSlow = 0.0254;

    public static final double algaeSafeDistTolerance = 0.127;

    public static final double ffMinRadius = 0.02;
    public static final double ffMaxRadius = 0.05;

    // Used to see if robot is up against reef or stuck against alliance partner during drive back
    public static final double notMovingVelocityThreshold = 0.0254;
    public static final double atReefFaceL1Tolerance = 0.08;
  }

  public static class Auto {
    public static final double algaeScoreDelay = 0.3;
  }

  public static class VisionObjectDetection {
    public static final String hostname = "VisionObjectDetectionCamera";
    public static final int numberOfGamePieceTypes =
        SimulatedGamePieceConstants.GamePieceType.values().length;
    public static final double trackedObjectToleranceMeters = 0.12;
    public static final Rotation2d lollipopTolerance = Rotation2d.fromDegrees(3.5);
    public static final boolean shouldIgnoreLollipopCoral = false;
    public static final Transform3d robotCenterToCamera =
        new Transform3d(
            -0.2208,
            -0.23495,
            0.98315,
            new Rotation3d(0, Units.degreesToRadians(40), Units.degreesToRadians(180)));
    public static final double coralIntakeOffset = Units.inchesToMeters(16); // TODO

    public static enum CoralIntakeMode {
      MANUAL,
      AUTO_ALIGN,
      AUTO_DRIVE,
      AUTO_ALIGN_DRIVE
    }

    public static final CoralIntakeMode coralIntakeMode = CoralIntakeMode.AUTO_ALIGN_DRIVE;
  }
}
