package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private ArmIO io;

  private static final double SAFE_ARM_ANGLE = 45.0;
  private static final double MIN_SAFE_ARM_ANGLE = 20.0;
  private static final double MAX_SAFE_ARM_ANGLE = 20.0;

  /* double currentPosition = arm.getPositionDeg();
  double requestedPosition = requestedPositionDeg.get();

  // First check if arm can move optimized
  if (currentPosition < 0) {
    if (requestedPosition > 0) {
      if (Math.abs(requestedPosition - currentPosition) > 180) {
        requestedPosition -= 360;
      }
    }
  } else if (currentPosition > 0) {
    if (requestedPosition < 0) {
      if (Math.abs(requestedPosition - currentPosition) > 180) {
        requestedPosition += 360;
      }
    }
  }

  if (requestedPosition > 270) {
    requestedPosition -= 360;
  } else if (requestedPosition < -270) {
    requestedPosition += 360;
  } */
  public Arm(ArmIO ArmIO) {
    this.io = ArmIO;
  }

  public Arm() {
    // TODO Auto-generated constructor stub
  }

  public enum Safety {
    SAFE,
    LOWERING,
    WAIT_FOR_ELEVATOR,
    MOVING_WITH_ELEVATOR,
    IDLE
  }

  Safety safety = Safety.SAFE;

  public void idle() {
    Nitrate.class.cast(io).setVoltage(0.0);
  }

  public void algaeHold() {}

  public void coralHold() {}

  public void algaeGround() {}

  public void algaeReef(/*requestedLevel*/ ) {}

  public void scoreAlgae(/*ArmState scoringSide*/ ) {}

  public void prescoreCoral(/*ArmState requestedLevel*/ ) {}

  public void scoreCoral(/*ArmState requestedLevel*/ ) {}

  public boolean atSetpoint() {
    // TODO: Implement logic to check if arm is at setpoint
    return true;
  }

  public void safeBargeRetract() {}

  public void setHome() {}

  public void setNeutralMode(/*ArmState mode*/ ) {}

  public void climbing() {}

  public void eject() {}

  public void getAngleDegrees() {}
}
