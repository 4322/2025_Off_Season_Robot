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

  public static class Drive {
    public static final int gyroID = 0; // TODO
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
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
