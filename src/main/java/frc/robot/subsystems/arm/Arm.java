package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private ArmIO io;

  public Arm(ArmIO ArmIO) {
    this.io = ArmIO;
  }

  private boolean armIdle;
  private boolean algaeHold;
  private boolean coralHold;
  private boolean algaeGround;
  private boolean algaeReef;
  private boolean scoreAlgae;
  private boolean prescoreCoral;
  private boolean scoreCoral;
  private boolean safeBargeRetract;
  private boolean getAngleDegrees;
  private boolean setHome;
  private boolean setNeutralMode;
  private boolean climbing;
  private boolean eject;

  public void armIdle() {
    unsetAllRequests();
    armIdle = true;
  }

  public void algaeHold() {
    unsetAllRequests();
    algaeHold = true;
  }

  public void coralHold() {
    unsetAllRequests();
    coralHold = true;
  }

  public void algaeGround() {
    unsetAllRequests();
    algaeGround = true;
  }

  public void algaeReef(/*requestedLevel*/ ) {
    unsetAllRequests();
    algaeReef = true;
  }

  public void scoreAlgae(/*ArmState scoringSide*/ ) {
    unsetAllRequests();
    scoreAlgae = true;
  }

  public void prescoreCoral(/*ArmState requestedLevel*/ ) {
    unsetAllRequests();
    prescoreCoral = true;
  }

  public void scoreCoral(/*ArmState requestedLevel*/ ) {
    unsetAllRequests();
    scoreCoral = true;
  }

  public boolean atSetpoint() {
    // TODO: Implement logic to check if arm is at setpoint
    return true;
  }

  public void safeBargeRetract() {
    unsetAllRequests();
    safeBargeRetract = true;
  }

  public void setHome() {
    unsetAllRequests();
    setHome = true;
  }

  public void setNeutralMode(/*ArmState mode*/ ) {
    unsetAllRequests();
    setNeutralMode = true;
  }

  public void climbing() {
    unsetAllRequests();
    climbing = true;
  }

  public void eject() {
    unsetAllRequests();
    eject = true;
  }

  public void getAngleDegrees() {}

  private void unsetAllRequests() {
    armIdle = false;
    algaeHold = false;
    coralHold = false;
    algaeGround = false;
    algaeReef = false;
    scoreAlgae = false;
    prescoreCoral = false;
    scoreCoral = false;
    safeBargeRetract = false;
    getAngleDegrees = false;
    setHome = false;
    setNeutralMode = false;
    climbing = false;
    eject = false;
  }
}
