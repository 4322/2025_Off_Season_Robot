package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.deployer.Deployer;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.rollers.Rollers;

public class IntakeSuperstructure extends SubsystemBase {

  private boolean requestHomed;
  private boolean requestRetractIdle;
  private boolean requestFeed;
  private boolean requestReject;
  private boolean requestIntakeEject;
  private boolean requestUnhome;

  private enum RetractLockedOutStates {
    FALSE,
    INDEXER,
    PICKUP,
  }

  private Timer retractTimeOutIndexerTimer = new Timer();
  private Timer retractTimeOutPickupAreaTimer = new Timer();
  private RetractLockedOutStates retractLockedOutState = RetractLockedOutStates.FALSE;

  private IntakeSuperstates state = IntakeSuperstates.UNHOMED;

  public Deployer deployer;
  private Rollers rollers;
  private Indexer indexer;

  public static enum IntakeSuperstates {
    UNHOMED,
    RETRACT_IDLE,
    FEED,
    SLOW_REJECT,
    INTAKE_EJECT
  }

  public IntakeSuperstructure(
      EndEffector endEffector, Deployer deployer, Rollers rollers, Indexer indexer) {
    this.deployer = deployer;
    this.rollers = rollers;
    this.indexer = indexer;
  }

  @Override
  public void periodic() {

    // The home button can only be activated when the robot is disabled, so accept it from any state
    if (requestHomed) {
      deployer.setHome();
      requestHomed = false;
      state = IntakeSuperstates.RETRACT_IDLE;
    }

    Logger.recordOutput("IntakeSuperstructure/State", state.toString());

    switch (state) {
      case UNHOMED:
        break;
      case RETRACT_IDLE:
      if (deployer.isDeployed()){
        deployer.deploy();
      }
      else {
        deployer.retract();}
        if (isCoralDetectedIndexer()
            || isCoralDetectedPickupArea()
            || RobotContainer.getSuperstructure().isCoralHeld()) {
          rollers.rejectSlow();
          indexer.rejectSlow();
        } else {
          rollers.feedSlow();
          indexer.feedSlow();
        }
        if (requestIntakeEject) {
          state = IntakeSuperstates.INTAKE_EJECT;
        }
        if (requestFeed) {
          state = IntakeSuperstates.FEED;
        }
        if (requestReject) {
          state = IntakeSuperstates.SLOW_REJECT;
        }
        break;
      case FEED:
        rollers.feed();
        indexer.feed();
        deployer.deploy();

        if (rollers.isCoralPickupDetected()
            && retractLockedOutState == RetractLockedOutStates.FALSE) {
          retractLockedOutState = RetractLockedOutStates.INDEXER;
          retractTimeOutIndexerTimer.reset();
          retractTimeOutIndexerTimer.start();
        } else if (isCoralDetectedIndexer()
            && retractLockedOutState == RetractLockedOutStates.INDEXER) {
          retractLockedOutState = RetractLockedOutStates.PICKUP;
          retractTimeOutIndexerTimer.stop();
          retractTimeOutPickupAreaTimer.reset();
          retractTimeOutPickupAreaTimer.start();
        } else if ((!isCoralDetectedIndexer() && !isCoralDetectedPickupArea())
            && retractTimeOutIndexerTimer.hasElapsed(
                Constants.IntakeSuperstructure.indexerRetractTimeoutSeconds)) {
          retractTimeOutIndexerTimer.stop();
          retractLockedOutState = RetractLockedOutStates.FALSE;
        } else if ((isCoralDetectedPickupArea()
                || retractTimeOutPickupAreaTimer.hasElapsed(
                    Constants.IntakeSuperstructure.pickupAreaRetractTimeoutSeconds))
            && retractLockedOutState == RetractLockedOutStates.PICKUP) {
          retractTimeOutPickupAreaTimer.stop();
          retractLockedOutState = RetractLockedOutStates.FALSE;
        }
        if (requestRetractIdle && retractLockedOutState == RetractLockedOutStates.FALSE) {
          state = IntakeSuperstates.RETRACT_IDLE;
        }
        if (isCoralDetectedIndexer()
            || isCoralDetectedPickupArea()
            || RobotContainer.getSuperstructure().isCoralHeld()) {
          state = IntakeSuperstates.SLOW_REJECT;
        }
        if (requestIntakeEject) {
          state = IntakeSuperstates.INTAKE_EJECT;
        }
        break;
      case SLOW_REJECT:
        deployer.deploy();
        rollers.reject();
        indexer.reject();

        if (!isCoralDetectedIndexer()
            && !isCoralDetectedPickupArea()
            && !RobotContainer.getSuperstructure().isCoralHeld()) {
          state = IntakeSuperstates.FEED;
        }
        if (requestRetractIdle) {
          state = IntakeSuperstates.RETRACT_IDLE;
        }
        if (requestIntakeEject) {
          state = IntakeSuperstates.INTAKE_EJECT;
        }
        break;
      case INTAKE_EJECT:
        deployer.eject();
        rollers.reject();
        indexer.reject();
        if (requestRetractIdle) {
          state = IntakeSuperstates.RETRACT_IDLE;
        }
        break;
    }
  }

  private void unsetAllRequests() {
    // don't clear requestHomed since it must be processed
    requestRetractIdle = false;
    requestFeed = false;
    requestReject = false;
    requestIntakeEject = false;
  }

  public IntakeSuperstates getState() {
    return state;
  }

  public void requestRetractIdle() {
    unsetAllRequests();
    requestRetractIdle = true;
  }

  public void requestIntake() {
    // Transitions to Feeding or Rejecting based on if coral in robot
    unsetAllRequests();
    if (isCoralDetectedPickupArea()) {
      requestReject = true;
    } else {
      requestFeed = true;
    }
  }

  public void requestEject() {
    unsetAllRequests();
    requestIntakeEject = true;
  }

  public void requestUnhome() {
    deployer.clearHome();
    rollers.idle();
    indexer.idle();
    state = IntakeSuperstates.UNHOMED;
    unsetAllRequests();
    requestUnhome = true;
  }

  public boolean isCoralDetectedPickupArea() {
    return indexer.isCoralDetectedPickupArea();
  }

  public boolean isCoralDetectedIndexer() {
    return indexer.isCoralDetectedIndexer();
  }

  public void homeButtonActivated() {
    requestHomed = true;
  }

  public boolean isCoralDetectedIntake() {
    return rollers.isCoralPickupDetected()
        || indexer.isCoralDetectedPickupArea()
        || indexer.isCoralDetectedIndexer();
  }
} // TODO check usage of reject vs. reject slow and eject vs eject slow
  // TODO add manual homing procedure
// Review logic
