package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;

public class ReefStatus {
  private boolean reefFaceAmbiguity;
  private boolean reefPipeAmbiguity;
  private Supplier<Rotation2d> closestRobotToReefFaceAngle;
  private ClosestReefPipe closestReefPipe;
  private L1Zone closestL1Zone;
  private Supplier<Integer> tagId;
  private AlgaeLevel algaeLevel;

  public enum L1Zone {
    MIDDLE,
    LEFT,
    RIGHT
  }

  public enum ClosestReefPipe {
    LEFT,
    RIGHT
  }

  public enum AlgaeLevel {
    L2,
    L3
  }

  public ReefStatus(
      boolean reefFaceAmbiguity,
      boolean reefPipeAmbiguity,
      Supplier<Rotation2d> closestReefFaceAngle,
      ClosestReefPipe closestReefPipe,
      L1Zone closestL1Zone,
      AlgaeLevel algaeLevel,
      Supplier<Integer> tagId) {
    this.reefFaceAmbiguity = reefFaceAmbiguity;
    this.reefPipeAmbiguity = reefPipeAmbiguity;
    this.closestRobotToReefFaceAngle = closestReefFaceAngle;
    this.closestReefPipe = closestReefPipe;
    this.closestL1Zone = closestL1Zone;
    this.tagId = tagId;
    this.algaeLevel = algaeLevel;
  }

  public ReefStatus(
      boolean reefFaceAmbiguity,
      boolean reefPipeAmbiguity,
      Rotation2d closestReefFaceAngle,
      ClosestReefPipe closestReefPipe,
      L1Zone closestL1Zone,
      AlgaeLevel algaeLevel,
      int tagId) {
    this.reefFaceAmbiguity = reefFaceAmbiguity;
    this.reefPipeAmbiguity = reefPipeAmbiguity;
    this.closestRobotToReefFaceAngle = () -> closestReefFaceAngle;
    this.closestReefPipe = closestReefPipe;
    this.closestL1Zone = closestL1Zone;
    this.tagId = () -> tagId;
    this.algaeLevel = algaeLevel;
  }

  public boolean getReefFaceAmbiguity() {
    return reefFaceAmbiguity;
  }

  public boolean getReefPipeAmbiguity() {
    return reefPipeAmbiguity;
  }

  public Rotation2d getClosestRobotAngle() {
    return closestRobotToReefFaceAngle.get();
  }

  public ClosestReefPipe getClosestReefPipe() {
    return closestReefPipe;
  }

  public L1Zone getClosestL1Zone() {
    return closestL1Zone;
  }

  public AlgaeLevel getAlgaeLevel() {
    return algaeLevel;
  }

  public int getFaceTagId() {
    return tagId.get();
  }
}
