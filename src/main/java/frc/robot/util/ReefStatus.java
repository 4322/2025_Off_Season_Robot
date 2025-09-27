package frc.robot.util;

public class ReefStatus {

  /*
  Boolean: Reef face ambiguity
  Boolean: Reef pipe ambiguity
  Object: Closest reef face
  Reef face angle
  Enum: Closest reef pipe
  Left, Right
  Enum: Closest L1 Zone
  Middle, Left, Right
   */

  public boolean getReefFaceAmbiguity() {
    return true;
  }
  // TODO

  public boolean getReefPipeAmbiguity() {
    return true; // TODO
  }

  public Object getClosestReefFace() {
    return null; // TODO
  }

  public enum ClosestReefPipe { // TODO
    LEFT,
    RIGHT
  }

  public enum L1Zone {
    MIDDLE,
    LEFT,
    RIGHT
  }
}
