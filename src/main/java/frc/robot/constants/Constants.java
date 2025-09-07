package frc.robot.constants;

import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.HardLimitConfig;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.InvertMode;
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
    public static final double robotMOI = 6.883; // TODO: Use CAD
    public static final double wheelCOF = 1.2;
  }

  public static class Scoring {
    public static final double scoringL1AngleDegCoral = 0.0;
    public static final double scoringL2AngleDegCoral = 17.5; // TODO: Set to actual angle
    public static final double scoringL3AngleDegCoral = 39.2; // TODO: Set to actual angle
    public static final double scoringL4AngleDegCoral = 37.2; // TODO: Set to actual angle
    public static final double maxElevatorSafeHeightMeters = 70; // need actual values
    public static final double scoreCoralL1HeightMeters = 0.3; // TODO: Set to actual position
    public static final double scoreCoralL2HeightMeters = 0.5; // TODO: Set to actual position
    public static final double scoreCoralL3HeightMeters = 0.7; // TODO: Set to actual position
    public static final double scoreCoralL4HeightMeters = 0.9; // TODO: Set to actual position
  }

  public static class Arm {
    public static final int armMotorId = 0; // TODO: Set to actual motor ID
    public static final int armEncoderId = 0; // TODO: Set to actual encoder ID

    public static final double sensorToArm = 56 / 16; // TODO: Set to actual gear ratio
    public static final double motorShaftToSensorShaft = 85 / 10; // TODO: Set to actual gear ratio

    public static final double armIdleDeg = 0.0;
    public static final double algaeHoldDeg = 180.0; // TODO: Set to actual angle
    public static final double coralHoldDeg = 150.0; // TODO: Set to actual angle
    public static final double algaeGroundDeg = 35.0; // TODO: Set to actual angle
    public static final double safeBargeRetractDeg = 100.0; // TODO: Set to actual angle

    public static final double ejectDeg = 45.0; // TODO: Set to actual angle
    public static final double climbingDeg = 25.0;

    // The purpose of
    public static final double minArmSafeDeg = 45.0;
    public static final double minArmSafeWithCoralDeg = 50.0;
    public static final double maxArmSafeAngle = 245.0;

    public static final double setpointToleranceDegrees = 0.01;
    public static final double supplyCurrentLimit = 40; // TODO
    public static final double statorCurrentLimit = 100; // TODO
    public static final double armFeedforward = 20;

    public static final ElectricalLimitSettings armElectricalLimitSettings =
        new ElectricalLimitSettings();

    public static final double scoringL1CoralDeg = 30;
    public static final double scoringL2CoralDeg = 0;
    public static final double scoringL3CoralDeg = 0;
    public static final double scoringL4CoralDeg = 90;
    public static final double prescoringL1CoralDeg = 70;
    public static final double prescoringL2CoralDeg = 110;
    public static final double prescoringL3CoralDeg = 100;
    public static final double prescoringL4CoralDeg = 120;
    public static final double scoringAlgaeDeg = 70; // TODO: Set to actual angle

    public static final double descoringAlgaeDeg = 90;
    public static final double safeBargeRetractAngleDeg = 180; // TODO: Set to actual angle
    // To the encoder 0 is horizontal but to us its straight down
    public static final double armOffsetEncoderDeg = -90;

    public static final double armkP = 0;
    public static final double armkI = 0;
    public static final double armkD = 0;
  }

  public static class Elevator {
    public static final int leftMotorID = 0; // TODO: Set to actual motor ID
    public static final int rightMotorID = 0; // TODO: Set to actual encoder ID
    public static final IdleMode motorIdleMode = IdleMode.kCoast;
    public static final InvertMode motorInvert = InvertMode.kInverted;
    public static final double elevatorMotorGearRatio = 1.0; // TODO: Set to actual gear ratio
    public static final PIDSettings elevatorMotorGains = new PIDSettings();
    public static final double kP = 0; // TODO: Set to actual value
    public static final double kI = 0; // TODO: Set to actual value
    public static final double kD = 0; // TODO: Set to actual value
    public static final double intializationVoltage = -0.1; // need actual values
    public static final double elevatorHeightToleranceMeters = 0.01;
    public static final double algaeHoldMeters = 0.1; // need actual values
    public static final double algaeGroundHeightMeters = 5; // need actual values
    public static final double algaeReefL1HeightMeters = 0.25; // TODO: Set to actual position
    public static final double algaeReefL2HeightMeters = 0.45; // TODO: Set to actual position
    public static final double algaeReefL3HeightMeters = 0.65; // TODO: Set to actual position
    public static final double prescoreCoralL1HeightMeters = 0.85; // TODO: Set to actual position
    public static final double prescoreCoralL2HeightMeters = 1.05; // TODO: Set to actual position
    public static final double prescoreCoralL3HeightMeters = 1.25; // TODO: Set to actual position
    public static final double prescoreCoralL4HeightMeters = 1.45; // TODO: Set to actual position
    public static final double pickupCoralHeightMeters = 0.5; // TODO: Set to actual position
    public static final double initializationTimerThresholdSecs = 0.01; //
    public static final double initializationVelocityMetersThresholdPerSecs = 0.01; //
    public static final double ejectSafeHeightMeters = 0.01; //
    public static final double safeBargeRetractHeightMeters = 0.01;
    public static final double supplyCurrentLimitAmps = 0.1;
    public static final double statorCurrentLimitAmps = 0.1;
    public static final double kG = 0;

    public static final double minElevatorSafeHeightMeters = 45.0;
    public static final double maxElevatorSafeHeightMeters = 100.0; // TODO: Set to actual height

    public static final double scoringL1CoralMeters = 10;
    public static final double scoringL2CoralMeters = 20;
    public static final double scoringL3CoralMeters = 30;
    public static final double scoringL4CoralMeters = 40;

    public static final double setpointToleranceDegrees = 0.01;
  }

  // TODO all of these are placeholder values
  public static class EndEffector {
    public static final int endEffectorMotorId = 0;
    public static final int endEffectorSensorId = 0;

    public static final double algaeHoldVolts = 1.0;
    public static final double coralHoldVolts = 1.0;

    public static final double algaeIntakeVolts = 3.0;
    public static final double coralIntakeVolts = 3.0;

    public static final double algaeReleaseVolts = -3.0;
    public static final double coralReleaseVolts =
        -3.0; // TODO make sure this is slow enough for scoring coral.

    public static final double ejectVolts = -3.0;

    public static final double motorBusCurrentLimit = 0;
    public static final double motorBusCurrentLimitTime = 0;
    public static final double motorStatorCurrentLimit = 0;

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
    public static final InvertMode motorInvert =
        InvertMode.kNotInverted; // InvertMode.kInverted or InvertMode.kNotInverted

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
    public static final int deployerMotorId = 1;
    public static final double deployVoltage = 3.0;

    public static final double motorStatorCurrentLimit = 0;
    public static final double motorBusCurrentLimitTime = 0;
    public static final double motorBusCurrentLimit = 0;
    public static final IdleMode motorIdleMode = IdleMode.kBrake;
    public static final InvertMode motorInvertMode = null;
    public static final HardLimitConfig motorForwardHardLimit = null;
    public static final HardLimitConfig motorReverseHardLimit = null;
    public static final double motorDeploykP = 1;
    public static final double motorDeploykI = 0;
    public static final double motorDeploykD = 0;
    public static final double motorDeployGravitationalFeedforward = 0;
    public static final double motorRetractkP = 1;
    public static final double motorRetractkI = 0;
    public static final double motorRetractkD = 0;
    public static final int deployerMotorEncoderId = 0;
    public static final boolean motorEncoderInverted = false; // TODO maybe change this?
    // 1 motor rotation is 1/49 of deployer rotation
    // Range of motion of deployer is about 0-140 degrees
    public static final double motorGearRatio = 49;
    public static final double ejectPositionRotations = Units.degreesToRotations(30);
    public static final double retractPositionRotations = Units.degreesToRotations(10);
    public static final double deployPositionRotations = Units.degreesToRotations(140);
  }

  public static class Indexer {
    public static final int indexerMotorId = 2;
    public static final double motorBusCurrentLimit = 0;
    public static final double motorBusCurrentLimitTime = 0;
    public static final double motorStatorCurrentLimit = 0;
    public static final IdleMode motorIdleMode = IdleMode.kCoast;
    public static final InvertMode motorInvert = null;
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
    public static final int rollersMotorId = 0;

    public static final double motorBusCurrentLimitTime = 0;
    public static final double motorStatorCurrentLimit = 0;
    public static final double motorBusCurrentLimit = 0;

    public static final IdleMode motorIdleMode = IdleMode.kCoast;
    public static final InvertMode motorInvert = null;

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
