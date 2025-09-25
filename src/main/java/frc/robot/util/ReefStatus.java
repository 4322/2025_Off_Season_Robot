package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

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

  public boolean ReefFaceAmbiguity(double Angle) {
    if (Angle == 0 || Angle == 60 || Angle == 120 || Angle == 180 || Angle == 240 || Angle == 300) {
      return false;
    } else {
      return true;
    }
  }
  // TODO

  public boolean ReefPipeAmbiguity(double Angle) {
    return true; // TODO
  }

  public Object ClosestReefFace(Rotation2d Angle) {
    return Angle; // TODO
  }

  public enum ReefPipe { // TODO
    LEFT,
    RIGHT
  }

  public enum L1Zone {
    MIDDLE,
    LEFT,
    RIGHT
  }

  public double ReefFaceAngle() {
    return 0; // TODO
  }
}
