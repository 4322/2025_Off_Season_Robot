package frc.robot.constants;

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
  public static final boolean armEnabled = true;
  public static final boolean elevatorEnabled = true;
  public static final boolean deployerEnabled = true;
  public static final boolean indexerEnabled = true;
  public static final boolean rollersEnabled = true;
  public static final boolean endEffectorEnabled = true;

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final boolean visionEnabled = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Scoring {
    public static final double L1ScoringAngleDegCoral = 0.0;
    public static final double L2ScoringAngleDegCoral = 17.5; // TODO: Set to actual angle
    public static final double L3ScoringAngleDegCoral = 39.2; // TODO: Set to actual angle
    public static final double L4ScoringAngleDegCoral = 37.2; // TODO: Set to actual angle
    public static final double AlgaePrescorePosition = 0.15; // TODO: Set to actual position
    public static final double SafeRetract = 0.240 - Units.inchesToMeters(2); // TODO
    public static final double safeFlipPosition = 0.217; // TODO
  }

  public static class Arm {
    public static final double IdlePositionDeg = 0.0; // TODO: Set to actual idle position
    public static final double kAlgaeHoldPositionDeg = 45.0; // TODO
    public static final double CoralHoldPositionDeg = 90.0; // TODO
    public static final double AlgaeGroundPositionDeg = 30.0; // TODO
    public static final double AlgaeReefPositionDeg = 60.0; // TODO
    public static final double ScoreAlgaePositionDeg = 75.0; // TODO

    public static final double setpointToleranceMeters = 0.01;

    public static final double supplyCurrentLimit = 40; // TODO
    public static final double statorCurrentLimit = 100; // TODO

    /*public static final int leftMotorID = 3; // follower motor
      public static final int rightMotorID = 2; // leader motor

      public static final double gearRatio = 4.0;
      public static final double sprocketDiameter = Units.inchesToMeters(1.751); // pitch diameter

      public static final double setpointToleranceMeters = 0.01;


      public static final double peakForwardVoltage = 11.5;
      public static final double peakReverseVoltage = -11.5;

      public static final InvertedValue rightMotorInversion = InvertedValue.Clockwise_Positive;

      // L1 height gains
      public static final double kS0 = 0;
      public static final double kP0 = 2.0;
      public static final double kD0 = 0.05;

      // L2 height gains
      public static final double kS1 = 0.4;
      public static final double kP1 = 2.0;
      public static final double kD1 = 0;

      // L3 height gains
      public static final double kS2 = 0;
      public static final double kP2 = 2.0;
      public static final double kD2 = 0.05;
      public static final double kG2 = 0.40;

      public static final double mechanismMaxAccel = 22; // Used for motion magic
      public static final double mechanismMaxCruiseVel = 3.2; // Used for motion magic
      public static final double motionMagicJerk = 0;

      public static final double homingVoltage = -1.0;
      public static final double homingVelocityThreshold = 0.01;
      public static final double homingThresholdSec = 0.25;

      public static final double jiggleHeight = 0.05;
    }

    public static class EndEffector {
      public static final int feederMotorID = 4;
      public static final int frontBeamBreakID = 8;
      public static final int backBeamBreakID = 9;

      public static final double supplyCurrentLimit = 40;
      public static final double statorCurrentLimit = 100;
      public static final InvertedValue motorInversion = InvertedValue.Clockwise_Positive;

      public static final double proximityDetectionThreshold = 0.075;

      public static final double feedVoltage = 4;
      public static final double secondFeedVoltage = 1.25;
      public static final double thirdFeedVoltage = -1.0;

      public static final double shootL1Voltage = 3;
      public static final double shootL23Voltage = 4;
      public static final double spitVoltage = -4;

      public static final double shootWaitTimerSec = 0.1;
      public static final double pullBackOverrideTimerSec = 1.0; */
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

    public static final double feedVoltage = 3.0;
    public static final double feedSlowVoltage = 3.0;
    public static final double ejectVoltage = 3.0;
    public static final double rejectVoltage = 3.0;
    public static final double rejectSlowVoltage = 3.0;
  }

  public static class IntakeSuperstructure {
    public static final double indexerRetractTimeoutSeconds = 3.0;
    public static final double pickupAreaRetractTimeoutSeconds = 3.0;
  }
}
