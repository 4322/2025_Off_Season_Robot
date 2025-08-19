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

  public static final boolean armEnabled = true;
  public static final boolean elevatorEnabled = true;
  public static final boolean deployerEnabled = true;
  public static final boolean indexerEnabled = true;
  public static final boolean rollersEnabled = true;
  public static final boolean endEffectorEnabled = true;

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final boolean visionEnabled = true;
  public static final boolean driveEnabled = true;

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
    public static final double algaePrescorePosition = 0.15; // TODO: Set to actual position
  }

  public static class Arm {
    public static final int armMotorId = 0; // TODO: Set to actual motor ID
    public static final int armEncoderId = 0; // TODO: Set to actual encoder ID

    public static final double armMotorGearRatio = 4.0; // TODO: Set to actual gear ratio

    // The purpose of
    public static final double minArmSafeDeg = 45.0;
    public static final double minArmSafeWithCoralDeg = 50.0;
    public static final double maxArmSafeAngle = 245.0;

    public static final double setpointToleranceDegrees = 0.01;
    public static final double supplyCurrentLimit = 40; // TODO
    public static final double statorCurrentLimit = 100; // TODO

    public static final ElectricalLimitSettings armElectricalLimitSettings =
        new ElectricalLimitSettings();

    public static final double scoringL1CoralDeg = 10;
    public static final double scoringL2CoralDeg = 20;
    public static final double scoringL3CoralDeg = 30;
    public static final double scoringL4CoralDeg = 40;

    public static final double descoringAlgaeDeg = 90;
    public static final double safeBargeRetractAngleDeg = 45; // TODO: Set to actual angle
    // To the encoder 0 is horizontal but to us its straight down
    public static final double armOffsetEncoderDeg = -90;

    public static final PIDSettings armMotorGains = new PIDSettings();
    public static final double kP =
        0; // TODO: Ask if I need multipule PID values depending on the Level
    public static final double kI = 0;
    public static final double kD = 0;
  }

  // TODO all of these are placeholder values
  public static class EndEffector {
    public static final int endEffectorMotorId = 0;
    public static final int endEffectorSensorId = 0;

    public static final double algaeHoldVolts = 3.0;
    public static final double coralHoldVolts = 3.0;

    public static final double algaeIntakeVolts = 3.0;
    public static final double coralIntakeVolts = 3.0;

    public static final double algaeReleaseVolts = 3.0;
    public static final double coralReleaseVolts = 3.0;

    public static final double currentDetectionThreshold = 0.0;

    public static final double motorBusCurrentLimit = 0;
    public static final double motorBusCurrentLimitTime = 0;
    public static final double motorStatorCurrentLimit = 0;

    public static final double sensorCoralProximityThreshold = 0;
    public static final double sensorAlgaeProximityThreshold = 0;

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
    public static final double ejectVolts = 0;
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
    public static final double motorDeploykP = 0;
    public static final double motorDeploykI = 0;
    public static final double motorDeploykD = 0;
    public static final double motorDeployGravitationalFeedforward = 0;
    public static final double motorRetractkP = 0;
    public static final double motorRetractkI = 0;
    public static final double motorRetractkD = 0;
    public static final int deployerMotorEncoderId = 0;
    public static final boolean motorEncoderInverted = false; // TODO maybe change this?
    public static final double motorGearRatio = 0;
    public static final double ejectPositionRotations = 0;
    public static final double retractPositionRotations = 0;
    public static final double deployPositionRotations = 0;
  }

  public static class Indexer {
    public static final int indexerMotorId = 2;
    public static final double feedVoltage = 3.0;
    public static final double feedSlowVoltage = 3.0;
    public static final double ejectVoltage = 3.0;
    public static final double rejectVoltage = 3.0;
    public static final double rejectSlowVoltage = 3.0;
    public static final double motorBusCurrentLimit = 0;
    public static final double motorBusCurrentLimitTime = 0;
    public static final double motorStatorCurrentLimit = 0;
    public static final IdleMode motorIdleMode = IdleMode.kCoast;
    public static final InvertMode motorInvert = null;
    public static final double indexerSensorMax = 0;
    public static final double pickupAreaSensorMax = 0;
    public static final int indexerSensorId = 0;
    public static final int pickupAreaSensorId = 0;
    public static final double motorVoltageFeed = 0;
    public static final double motorVoltageRejectSlow = 0;
    public static final double motorVoltageFeedSlow = 0;
    public static final double motorVoltageEject = 0;
    public static final double motorVoltageEjectSlow = 0;
    public static final double motorVoltageReject = 0;
  }

  public static class Rollers {
    public static final int rollersMotorID = 3;

    public static final int rollersMotorId = 0;

    public static final double motorBusCurrentLimitTime = 0;

    public static final double motorStatorCurrentLimit = 0;

    public static final double motorBusCurrentLimit = 0;

    public static final IdleMode motorIdleMode = IdleMode.kCoast;

    public static final InvertMode motorInvert = null;

    public static final double motorVoltageFeed = 0;

    public static final double motorVoltageFeedSlow = 0;

    public static final double motorVoltageReject = 0;

    public static final double motorVoltageRejectSlow = 0;

    public static final double motorVoltageEject = 0;
  }

  public static class IntakeSuperstructure {
    public static final double indexerRetractTimeoutSeconds = 3.0;
    public static final double pickupAreaRetractTimeoutSeconds = 3.0;
  }

  public static class Elevator {
    public static final double minElevatorSafeHeightMeters = 45.0;
    public static final double maxElevatorSafeHeightMeters = 100.0; // TODO: Set to actual height
   
    public static final double scoringL1CoralMeters = 10;
    public static final double scoringL2CoralMeters = 20;
    public static final double scoringL3CoralMeters = 30;
    public static final double scoringL4CoralMeters = 40;
  } // TODO placeholder values
}
