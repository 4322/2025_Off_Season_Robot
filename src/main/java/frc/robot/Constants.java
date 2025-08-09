package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final boolean armEnabled = true;
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

  public static class PathPlanner {
    public static final double translationkP = 0;
    public static final double translationkD = 0;

    public static final double rotkP = 0.0;
    public static final double rotkD = 0.0;

    public static final double robotMassKg = 74.088; // TODO: Weigh robot
    public static final double robotMOI = 6.883; // TODO: Use CAD
    public static final double wheelCOF = 1.2;
  }

  public static class Arm {
    public static final double kIdlePositionDeg = 0.0; // TODO: Set to actual idle position
    public static final double kAlgaeHoldPositionDeg = 45.0; // TODO
    public static final double CoralHoldPositionDeg = 90.0; // TODO
    public static final double AlgaeGroundPositionDeg = 30.0;
    public static final double AlgaeReefPositionDeg = 60.0;
    public static final double ScoreAlgaePositionDeg = 75.0;
    public static final double PrescoreCoralPositionDeg = 80.0;
    public static final double ScoreCoralPositionDeg = 100.0;
  }
}
