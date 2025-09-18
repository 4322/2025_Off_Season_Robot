package frc.robot.constants;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.InvertMode;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDFeedforwardMode;
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

  public static final boolean armEnabled = false;
  public static final boolean elevatorEnabled = false;
  public static final boolean deployerEnabled = false;
  public static final boolean indexerEnabled = false;
  public static final boolean rollersEnabled = false;
  public static final boolean endEffectorEnabled = false;
  public static final boolean visionEnabled = false;
  public static final boolean driveEnabled = false;

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  public static final String logPath = "/home/lvuser/logs";
  public static final long minFreeSpace = 1000000000; // 1 GB

  public static final int dioHomeButton = 0;
  public static final double homeButtonDelaySec = 1.0;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Drive {
    public static final double autoRotatekP = 5;
    public static final double autoRotatekD = 0;

    public static final double angularErrorToleranceRad = Units.degreesToRadians(7);
    public static final double angularErrorToleranceRadPerSec = Units.degreesToRadians(20.0);
    public static final double driveDeadband = 0.1;
    public static final double rotDeadband = 0.1;

    public static final double pseudoAutoRotatekP = 6;
    public static final double pseudoAutoRotatekI = 0;
    public static final double pseudoAutoRotatekD = 0.0;
    public static final double pseudoAutoRotateRadTolerance = Units.degreesToRadians(1.5);
    public static final double inhibitPseudoAutoRotateRadPerSec = Units.degreesToRadians(4);
    public static final double pseudoAutoRotateMinMetersPerSec =
        0.6; // disable below this speed for fine adjustments
  }

  public static class PathPlanner {
    public static final double translationkP = 0;
    public static final double translationkD = 0;

    public static final double rotkP = 0.0;
    public static final double rotkD = 0.0;

    public static final double robotMassKg = 74.088; // TODO: Weigh robot
    public static final double robotMOI =
        Units.lbsToKilograms(Units.inchesToMeters(402.462753)); // -Lxy
    public static final double wheelCOF = 1.2;
  }

  public static class Arm {
    public static final int armMotorId = 10;
    public static final int armEncoderId = 10;

    public static final boolean manualControl = false;
    public static final InvertMode motorInvert =
        InvertMode.kNotInverted; // positive is up toward scoring side
    public static final IdleMode motorIdleMode = IdleMode.kBrake;

    public static final double sensorToArm = 85 / 10.0;
    public static final double motorShaftToSensorShaft = 56 / 16.0;

    public static final double armIdleDeg = 0.0;
    public static final double algaeHoldDeg = 180.0;
    public static final double coralHoldDeg = armIdleDeg;
    public static final double algaeGroundDeg = 55.0; // TODO: Set to actual angle

    public static final double ejectDeg = 45.0;
    public static final double climbingDeg = 25.0; // TODO: Set to actual angle

    public static final double minArmSafeDeg = 45.0; // TODO: Set to actual angle
    public static final double minArmSafeWithCoralDeg = 55.0; // TODO: Set to actual angle
    public static final double maxArmSafeDeg = 245.0;

    public static final double setpointToleranceDegrees = 0.5;
    public static final double bufferDeg =
        setpointToleranceDegrees * 4; // Degrees of buffer zone for min safe angle
    public static final double supplyCurrentLimitAmps = 40;
    public static final double statorCurrentLimitAmps = 100;
    public static final double kG = 20;

    public static final double scoringL1CoralDeg = Constants.Arm.prescoringL1CoralDeg - 5; // TODO
    public static final double scoringL2CoralDeg = Constants.Arm.prescoringL2CoralDeg - 20; // TODO
    public static final double scoringL3CoralDeg = Constants.Arm.prescoringL3CoralDeg - 20; // TODO
    public static final double scoringL4CoralDeg = Constants.Arm.prescoringL3CoralDeg - 30; // TODO

    // Prescore Degrees Arm
    public static final double prescoringL1CoralDeg = 50.686373;
    public static final double prescoringL2CoralDeg = 130.751475;
    public static final double prescoringL3CoralDeg = 125.970093;
    public static final double prescoringL4CoralDeg = 121.294978;
    public static final double scoringAlgaeDeg = 139.326425;

    public static final double descoringAlgaeDeg = 81.946341;
    public static final double safeBargeRetractDeg = 180;

    // To the encoder 0 is horizontal but to us its straight down
    public static final double OffsetEncoderDeg = -90;

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double AccelerationLimit = 0.68; // TODO
    public static final double DeaccelerationLimit = 0.68; // TODO
    public static final double VelocityLimit = 1.7; // TODO
    public static final double slowVelocityLimit = 0.6; // TODO
  }

  public static class Elevator {
    public static final int frontMotorID = 20;
    public static final int backMotorID = 21;

    public static final boolean manualControl = false;
    public static final IdleMode motorIdleMode = IdleMode.kBrake;
    public static final InvertMode motorFrontInvert = InvertMode.kNotInverted; // positive is up
    public static final InvertMode motorBackInvert = InvertMode.kInverted; // positive is up
    public static final double fast_kP = 1; // TODO: Set to actual value
    public static final double fast_kI = 0; // TODO: Set to actual value
    public static final double fast_kD = 0; // TODO: Set to actual value
    public static final double slow_kP = 1; // TODO: Set to actual value
    public static final double slow_kI = 0; // TODO: Set to actual value
    public static final double slow_kD = 0; // TODO: Set to actual value
    public static final double kG = 1; // TODO: Set to actual value
    public static final double maxElevatorHeightMeters = 1.3068401092;
    public static final double homeHeightMeters = 0.3917895136;
    public static final double minElevatorSafeHeightMeters = homeHeightMeters + 0.1;
    public static final double minElevatorSafeWithCoralHeightMeters = homeHeightMeters + 0.3;
    public static final double elevatorHeightToleranceMeters = 0.01;
    public static final double algaeGroundHeightMeters = 0.2; // TODO: need actual value
    public static final double algaeReefL2HeightMeters = 0.5021688204;
    public static final double algaeReefL3HeightMeters = 0.8739758746;
    public static final double prescoreCoralL1HeightMeters = 0.5144884808;
    public static final double prescoreCoralL2HeightMeters = 0.012381357;
    public static final double prescoreCoralL3HeightMeters = 0.3703222972;
    public static final double prescoreCoralL4HeightMeters = maxElevatorHeightMeters - 0.00635;
    public static final double pickupCoralHeightMeters = homeHeightMeters + 0.01;
    public static final double intializationVoltage = 2.0;
    public static final double initializationTimerThresholdSecs = 5;
    public static final double initializationVelocityMetersThresholdPerSecs = 0.01;
    public static final double ejectHeightMeters = minElevatorSafeWithCoralHeightMeters;
    public static final double safeBargeRetractHeightMeters = 0.773472037;
    public static final double safeBargeRetractWithAlgaeHeightMeters = 0.3321799808;
    public static final double algaeHoldMeters = prescoreCoralL3HeightMeters;

    public static final double supplyCurrentLimitAmps = 40;
    public static final double statorCurrentLimitAmps = 100;
    public static final double fastAccelerationMetersPerSec2 = 4.0;
    public static final double fastDecelerationMetersPerSec2 = 4.0;
    public static final double fastVelocityMetersPerSec = 1.0;
    public static final double slowAccelerationMetersPerSec2 = 0.8;
    public static final double slowDecelerationMetersPerSec2 = 0.8;
    public static final double slowVelocityMetersPerSec = 0.2;

    public static final double gearRatio = 6 / 1.0;
    public static final double scoreCoralL1HeightMeters = prescoreCoralL1HeightMeters;
    public static final double scoreCoralL2HeightMeters = prescoreCoralL2HeightMeters - 0.1;
    public static final double scoreCoralL3HeightMeters = prescoreCoralL3HeightMeters - 0.1;
    public static final double scoreCoralL4HeightMeters = prescoreCoralL4HeightMeters - 0.1;
    public static final double scoreAlgaeHeightMeters = maxElevatorHeightMeters - 0.00635;

    public static final double bufferHeightMeters = elevatorHeightToleranceMeters * 2;
  }

  // TODO all of these are placeholder values
  public static class EndEffector {
    public static final int endEffectorMotorId = 30;
    public static final int endEffectorSensorId = 30;

    public static final double algaeHoldVolts = 1.0;
    public static final double coralHoldVolts = 1.0;

    public static final double algaeIntakeVolts = 3.0;
    public static final double coralIntakeVolts = 3.0;

    public static final double algaeReleaseVolts = -3.0;
    public static final double coralReleaseVolts =
        -3.0; // TODO make sure this is slow enough for scoring coral.
    public static final double coralReleaseVoltsL1 = -2.0;

    public static final double ejectVolts = -3.0;

    public static final double motorBusCurrentLimit = 40;
    public static final double motorBusCurrentLimitTime = 40;
    public static final double motorStatorCurrentLimit = 60;

    public static final double sensorCoralProximityThreshold = 0;
    public static final double sensorAlgaeProximityThreshold = 0;

    public static final boolean useSensorColor = false; // TODO change this when we get color tuned
    // TODO tune these
    // For algae
    public static final double sensorGreenDetectGreenLower = 120;
    public static final double sensorGreenDetectGreenUpper = 140;
    public static final double sensorGreenDetectBlueLower = 120;
    public static final double sensorGreenDetectBlueUpper = 140;
    public static final double sensorGreenDetectRed = 38; // Max value

    // For coral; All are minimum values
    public static final double sensorWhiteDetectGreen = 180;
    public static final double sensorWhiteDetectBlue = 180;
    public static final double sensorWhiteDetectRed = 180;
    public static final IdleMode motorIdleMode = IdleMode.kBrake;
    public static final InvertMode motorInvert = InvertMode.kNotInverted; // positive is intaking

    // TODO tune these
    public static final double currentDetectionDebounceTimeSeconds =
        0.25; // Time for the current to spike and stay there before detection is triggered
    public static final double velocityDetectionDebounceTimeSeconds =
        0.25; // Time for veleocity to spike and stay there before detection is triggered
    public static final double CurrentDetectionDeltaThresholdAmps = 0;
    public static final double CurrentDetectionMaxAccumulationSeconds = 1;
    public static final double VelocityDetectionMaxAccumulationSeconds = 1;
    public static final double VelocityDetectionDeltaThresholdRotationsPerSecond = 0;

    public static final double algaeIntakingDelaySeconds =
        0.05; // Time to wait after algae is detected in End Effector before reducing voltage
    public static final double coralIntakingDelaySeconds =
        0.05; // Time to wait after coral is detected in End Effector before reducing voltage
  }

  public static class Deployer {
    public static final int deployerMotorId = 40;
    public static final double deployVoltage = 3.0;

    public static final double motorStatorCurrentLimit = 40;
    public static final double motorBusCurrentLimitTime = 40;
    public static final double motorBusCurrentLimit = 60;
    public static final IdleMode motorIdleMode = IdleMode.kBrake;
    public static final InvertMode motorInvertMode =
        InvertMode.kNotInverted; // positive is retracting
    public static final double motorDeploykP = 1;
    public static final double motorDeploykI = 0;
    public static final double motorDeploykD = 0;
    public static final double motorDeployGravitationalFeedforward = 0;
    public static final double motorRetractkP = 1;
    public static final double motorRetractkI = 0;
    public static final double motorRetractkD = 0;
    public static final int deployerMotorEncoderId = 0; // not currently installed
    public static final InvertMode motorEncoderInverted =
        InvertMode.kInverted; // reverse of motor, if installed
    // 1 motor rotation is 1/49 of deployer rotation
    // Range of motion of deployer is about 0-140 degrees
    public static final double motorGearRatio = 14.58333333333;
    public static final double ejectPositionRotations = Units.degreesToRotations(94.931222);
    public static final double retractPositionRotations = Units.degreesToRotations(4.931222);
    public static final double deployPositionRotations = Units.degreesToRotations(145.353984);
    public static final PIDFeedforwardMode motorFeedforwardMode = PIDFeedforwardMode.kStandard;
    public static final double motorOffsetDegrees = 145.353984;
  }

  public static class Indexer {
    public static final int indexerMotorRightId = 50;
    public static final int indexerMotorLeftId = 51;
    public static final double motorBusCurrentLimit = 40;
    public static final double motorBusCurrentLimitTime = 0.5;
    public static final double motorStatorCurrentLimit = 60;
    public static final IdleMode motorIdleMode = IdleMode.kCoast;
    public static final InvertMode motorRightInvert = InvertMode.kInverted; // positive is intaking
    public static final InvertMode motorLeftInvert =
        InvertMode.kNotInverted; // positive is intaking
    public static final double indexerSensorMax = 0;
    public static final double pickupAreaSensorMax = 0;
    public static final int indexerSensorId = 0;
    public static final int pickupAreaSensorId = 0;
    public static final double motorVoltageFeed = 5;
    public static final double motorVoltageRejectSlow = -3;
    public static final double motorVoltageFeedSlow = 3;
    public static final double motorVoltageReject = -5;
  }

  public static class Rollers {
    public static final int rollersMotorId = 60;

    public static final double motorBusCurrentLimitTime = 0.5;
    public static final double motorStatorCurrentLimit = 60;
    public static final double motorBusCurrentLimit = 40;

    public static final IdleMode motorIdleMode = IdleMode.kCoast;
    public static final InvertMode motorInvert =
        InvertMode.kNotInverted; // positive is intaking, TODO: verify direction

    public static final double motorVoltageFeed = 5;
    public static final double motorVoltageFeedSlow = 3;
    public static final double motorVoltageReject = -5;
    public static final double motorVoltageRejectSlow = -3;
    // TODO tune these
    public static final double currentDetectionDebounceTimeSeconds =
        0.25; // Time for the delta of the current to spike and stay there before detection is
    // triggered
    public static final double velocityDetectionDebounceTimeSeconds =
        0.25; // Time for delta of the velocity to spike and stay there before detection is
    // triggered

    public static final double CurrentDetectionDeltaThresholdAmps = 0;
    public static final double velocityDetectionStallDeltaRotationsPerSec = 0;
    public static final double CurrentDetectionMaxAccumulationSeconds = 1;
    public static final double VelocityDetectionDeltaThresholdRotationsPerSecond = 0;
    public static final double VelocityDetectionMaxAccumulationSeconds = 1;
  }

  public static class IntakeSuperstructure {
    public static final double indexerRetractTimeoutSeconds = 3.0;
    public static final double pickupAreaRetractTimeoutSeconds = 3.0;
  }
}
