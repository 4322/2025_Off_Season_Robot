package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class ReefStatus {
  private boolean reefFaceAmbiguity;
  private boolean reefPipeAmbiguity;
  private Rotation2d closestReefFaceAngle;
  private ClosestReefPipe closestReefPipe;
  private L1Zone closestL1Zone;

  public enum L1Zone {
    MIDDLE,
    LEFT,
    RIGHT
  }

  public enum ClosestReefPipe {
    LEFT,
    RIGHT
  }

  public ReefStatus(
      boolean reefFaceAmbiguity,
      boolean reefPipeAmbiguity,
      Rotation2d closestReefFaceAngle,
      ClosestReefPipe closestReefPipe,
      L1Zone closestL1Zone) {
    this.reefFaceAmbiguity = reefFaceAmbiguity;
    this.reefPipeAmbiguity = reefPipeAmbiguity;
    this.closestReefFaceAngle = closestReefFaceAngle;
    this.closestReefPipe = closestReefPipe;
    this.closestL1Zone = closestL1Zone;
  }

  public boolean getReefFaceAmbiguity() {
    return reefFaceAmbiguity;
  }

  public boolean getReefPipeAmbiguity() {
    return reefPipeAmbiguity;
  }

  public Rotation2d getClosestReefFaceAngle() {
    return closestReefFaceAngle;
  }

  public ClosestReefPipe getClosestReefPipe() {
    return closestReefPipe;
  }

  public L1Zone getClosestL1Zone() {
    return closestL1Zone;
  }
}
