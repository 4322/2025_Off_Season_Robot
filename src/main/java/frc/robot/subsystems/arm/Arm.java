package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private TunnelIO io;
  private TunnelIOInputsAutoLogged inputs = new TunnelIOInputsAutoLogged();

  private boolean armIdle;

  private Arm() {
    switch (Constants.currentMode) {
      case REAL:
        if (Constants.armEnabled) {
          io = new ArmIOTalonFX();
        }
        break;
      case SIM:
        break;
      case REPLAY:
        break;
    }

    if (io == null) {
      io = new ArmIO() {};
    }
  }

  public enum ArmState {
    STARTING_CONFIG,
    HOMING,
    REQUEST_SETPOINT,
    JIGGLE
  }

  ArmState currentState = ArmState.STARTING_CONFIG;

  @Override
  public void periodic() {
    switch (currentState) {
      case STARTING_CONFIG:
        // Handle starting config logic
        break;
      case HOMING:
        // Handle homing logic
        break;
      case REQUEST_SETPOINT:
        // Handle request setpoint logic
        break;
      case JIGGLE:
        // Handle jiggle logic
        break;
    }
  }

  /* idle()
  algaeHold()
  coralHold()
  algaeGround()
  algaeReef(Level requestedLevel)
  scoreAlgae(Side scoringSide)
  prescoreCoral(Level requestedLevel)
  scoreCoral(Level requestedLevel)
  atSetpoint()
  safeBargeRetract()
  getAngleDegrees()
  setHome()
  setNeutralMode(NeutralMode mode)
  climbing()
  eject() */

  public void armIdle() {
    unsetAllRequests();
    armIdle = true;
  }

  public void algaeHold() {
    unsetAllRequests();
  }

  public void coralHold() {
    unsetAllRequests();
  }

  public void algaeGround() {
    unsetAllRequests();
  }

  public void algaeReef(ArmState requestedLevel) {
    unsetAllRequests();
  }

  public void scoreAlgae(ArmState scoringSide) {
    unsetAllRequests();
  }

  public void prescoreCoral(ArmState requestedLevel) {
    unsetAllRequests();
  }

  public void scoreCoral(ArmState requestedLevel) {
    unsetAllRequests();
  }

  public boolean atSetpoint() {
    // TODO: Implement logic to check if arm is at setpoint
    return true;
  }

  public void safeBargeRetract() {
    unsetAllRequests();
  }

  public void setHome() {
    unsetAllRequests();
  }

  public void setNeutralMode(ArmState mode) {
    unsetAllRequests();
  }

  public void climbing() {
    unsetAllRequests();
  }

  public void eject() {
    unsetAllRequests();
  }

  public void getAngleDegrees() {}

  private void unsetAllRequests() {
    armIdle = false;
  }
}
