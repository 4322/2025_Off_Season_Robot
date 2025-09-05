package frc.robot.subsystems.elevator;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.util.ClockUtil;

public class Elevator extends SubsystemBase {
  private boolean elevatorIdle;
  private ElevatorIO io;
  private Timer homingTimer = new Timer();
  ElevatorStates state = ElevatorStates.STARTING_CONFIG;
  ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double requestedHeightMeters = 0.0;

  private enum ElevatorStates {
    STARTING_CONFIG,
    HOMING,
    REQUEST_SETPOINT,
    JIGGLE
  }

  public Elevator(ElevatorIO ELVIO) {
    this.io = ELVIO;
  }

  @Override
  public void periodic() {
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
  public void idle() {}

  public void algaeHold() {}

  public void coralHold() {}

  public void algaeGround() {}

  public void algaeReef(Level level) {
    // requestElevator = true;
    switch (level) {
      case L1:
        requestedHeightMeters = Constants.Elevator.algaeReefL1HeightMeters;
        break;
      case L2:
        requestedHeightMeters = Constants.Elevator.algaeReefL2HeightMeters;
        break;
      case L3:
        requestedHeightMeters = Constants.Elevator.algaeReefL3HeightMeters;
        break;
    }
  }

  public void scoreAlgae() {
    // equestElevator = true;
    requestedHeightMeters = Constants.Scoring.maxElevatorSafeHeightMeters;
  }

  public void prescoreCoral(Level level) {
    switch (level) {
      case L1:
        requestedHeightMeters = Constants.Elevator.prescoreCoralL1HeightMeters;
        break;
      case L2:
        requestedHeightMeters = Constants.Elevator.prescoreCoralL2HeightMeters;
        break;
      case L3:
        requestedHeightMeters = Constants.Elevator.prescoreCoralL3HeightMeters;
        break;
      case L4:
        requestedHeightMeters = Constants.Elevator.prescoreCoralL4HeightMeters;
        break;
    }
  }

  public void scoreCoral(Level level) {
    switch (level) {
      case L1:
        requestedHeightMeters = Constants.Scoring.scoreCoralL1HeightMeters;
        break;
      case L2:
        requestedHeightMeters = Constants.Scoring.scoreCoralL2HeightMeters;
        break;
      case L3:
        requestedHeightMeters = Constants.Scoring.scoreCoralL3HeightMeters;
        break;
      case L4:
        requestedHeightMeters = Constants.Scoring.scoreCoralL4HeightMeters;
        break;
    }
  }

  public void pickupCoral() {}

  public boolean atSetpoint() {
    return ClockUtil.atReference(
        inputs.heightMeters,
        requestedHeightMeters,
        Constants.Elevator.setpointToleranceDegrees,
        true);
  }

  public double getHeightMeters() {
    return inputs.heightMeters;
  }

  public void setManualInitialization() {}

  public void setNeutralMode(IdleMode mode) {}

  public void safeBargeRetract() {}

  public void climbing() {}

  public void eject() {}
}
