package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private boolean elevatorIdle;
  private ElevatorIO io;
  private Timer homingTimer = new Timer();
  ElevatorStates state = ElevatorStates.STARTING_CONFIG;

  private enum ElevatorStates {
    STARTING_CONFIG,
    HOMING,
    REQUEST_SETPOINT,
    JIGGLE
  }

  private Elevator() {
    switch (state) {
      case STARTING_CONFIG:
        // Handle starting config logic
        break;
      case HOMING:
        // Handle homing logic
        homingTimer.start();
        break;
      case REQUEST_SETPOINT:
        // Handle request setpoint logic
        break;
      case JIGGLE:
        // Handle jiggle logic
        break;
    }
  }

  /*idle()
  algaeHold()
  coralHold()
  algaeGround()
  algaeReef(Level requestedLevel)
  scoreAlgae()
  prescoreCoral(Level requestedLevel)
  scoreCoral(Level requestedLevel)
  pickupCoral()
  atSetpoint()
  getHeightMeters()
  setHome()
  setNeutralMode(NeutralMode mode)
  safeBargeRetract()
  climbing()
  eject()
   */
  public void ElevatorIdle() {}

  public void algaeHold() {}

  public void coralHold() {}

  public void algaeGround() {}

  public void algaeReef(/*Level level*/ ) {}

  public void scoreAlgae() {}

  public void prescoreCoral(/*Level level*/ ) {}

  public void scoreCoral(/*Level level*/ ) {}

  public void pickupCoral() {}

  public boolean atSetpoint() {
    return false;
  }

  public double getHeightMeters() {
    return 0.0;
  }

  public void setHome() {}

  public void setNeutralMode(/*State mode*/ ) {}

  public void safeBargeRetract() {}

  public void climbing() {}

  public void eject() {}
}
