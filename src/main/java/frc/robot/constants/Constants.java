package frc.robot.constants;

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
    public static final int endEffectorMotorID = 0;
    public static final int endEffectorSensorID = 0;

    public static final double ALGAE_HOLD_VOLTS = 3.0;
    public static final double CORAL_HOLD_VOLTS = 3.0;

    public static final double ALGAE_INTAKE_VOLTS = 3.0;
    public static final double CORAL_INTAKE_VOLTS = 3.0;

    public static final double ALGAE_RELEASE_VOLTS = 3.0;
    public static final double CORAL_RELEASE_VOLTS = 3.0;

    public static final double CURRENT_DETECTION_THRESHOLD = 0.0;

    public static final double MOTOR_BUS_CURRENT_LIMIT = 0;
    public static final double MOTOR_BUS_CURRENT_LIMIT_TIME = 0;
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 0;

    public static final double SENSOR_CORAL_PROXIMITY_THRESHOLD = 0;
    public static final double SENSOR_ALGAE_PROXIMITY_THRESHOLD = 0;


    // TODO tune these
    // For algae
    public static final double SENSOR_GREEN_DETECT_GREEN_LOWER= 120;
    public static final double SENSOR_GREEN_DETECT_GREEN_UPPER = 140;
    public static final double SENSOR_GREEN_DETECT_BLUE_LOWER = 120;
    public static final double SENSOR_GREEN_DETECT_BLUE_UPPER = 140;
    public static final double SENSOR_GREEN_DETECT_RED = 38; // Max value
    // For coral; All are minimum values
    public static final double SENSOR_WHITE_DETECT_GREEN = 180;
    public static final double SENSOR_WHITE_DETECT_BLUE = 180;
    public static final double SENSOR_WHITE_DETECT_RED = 180;
  }

  public static class Deployer {
    public static final int deployerMotorID = 1;
    public static final double DEPLOY_VOLTAGE = 3.0;
  }

  public static class Indexer {
    public static final int indexerMotorID = 2;
    public static final double FEED_VOLTAGE = 3.0;
    public static final double FEED_SLOW_VOLTAGE = 3.0;
    public static final double EJECT_VOLTAGE = 3.0;
    public static final double REJECT_VOLTAGE = 3.0;
    public static final double REJECT_SLOW_VOLTAGE = 3.0;
  }

  public static class Rollers {
    public static final int rollersMotorID = 3;

    public static final double FEED_VOLTAGE = 3.0;
    public static final double FEED_SLOW_VOLTAGE = 3.0;
    public static final double EJECT_VOLTAGE = 3.0;
    public static final double REJECT_VOLTAGE = 3.0;
    public static final double REJECT_SLOW_VOLTAGE = 3.0;
  }

  public static class IntakeSuperstructure {
    public static final double INDEXER_RETRACT_TIMEOUT_SECONDS = 3.0;
    public static final double PICKUP_AREA_RETRACT_TIMEOUT_SECONDS = 3.0;
  }
}
