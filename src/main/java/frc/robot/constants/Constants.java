package frc.robot.constants;

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

  public static class IntakeSuperstructure {
    
    public static final double indexerRetractTimeoutSeconds = 3; // TODO placeholder values
    public static final double pickupAreaRetractTimeoutSeconds = 3; 
  }
}
