package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.deployer.Deployer;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.rollers.Rollers;
import org.littletonrobotics.junction.Logger;

public class IntakeSuperstructure extends SubsystemBase {

  private boolean requestRetractIdle;
  private boolean requestFeed;
  private boolean requestReject;
  private boolean requestIntakeEject;

  private enum RetractLockedOutStates {
    FALSE,
    INDEXER,
    PICKUP,
  }

  private Timer retractTimeOutIndexerTimer = new Timer();
  private Timer retractTimeOutPickupAreaTimer = new Timer();
  private RetractLockedOutStates retractLockedOutState = RetractLockedOutStates.FALSE;

  private IntakeSuperstates state = IntakeSuperstates.START;

  private Deployer deployer;
  private Rollers rollers;
  private Indexer indexer;

  public static enum IntakeSuperstates {
    START,
    RETRACT_IDLE,
    FEED,
    REJECT,
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
    Logger.recordOutput("IntakeSuperstructure/State", state.toString());
    switch (state) {
        // TODO update this with new homing logic
      case START:
        if (RobotContainer.superstructure.isHomeButtonPressed()) {
          deployer.setHome();
          state = IntakeSuperstates.RETRACT_IDLE;
        }
        break;
      case RETRACT_IDLE:
        deployer.retract();

        if (isCoralDetectedIndexer()
            || isCoralDetectedPickupArea()
            || RobotContainer.superstructure.isCoralHeld()) {
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
          state = IntakeSuperstates.REJECT;
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
            || RobotContainer.superstructure.isCoralHeld()) {
          state = IntakeSuperstates.REJECT;
        }
        if (requestIntakeEject) {
          state = IntakeSuperstates.INTAKE_EJECT;
        }
        break;
      case REJECT:
        deployer.deploy();
        rollers.reject();
        indexer.reject();

        if (!isCoralDetectedIndexer()
            && !isCoralDetectedPickupArea()
            && !RobotContainer.superstructure.isCoralHeld()) {
          state = IntakeSuperstates.FEED;
        }
        if (requestRetractIdle) {
          state = IntakeSuperstates.RETRACT_IDLE;
        }
        break;
      case INTAKE_EJECT:
        deployer.ejectPosition();
        rollers.eject();
        indexer.eject();
        if (requestRetractIdle) {
          state = IntakeSuperstates.RETRACT_IDLE;
        }
        break;
    }
  }

  private void unsetAllRequests() {
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
    requestReject = true;
  }

  public boolean isCoralDetectedPickupArea() {
    return indexer.isCoralDetectedPickupArea();
  }

  public boolean isCoralDetectedIndexer() {
    return indexer.isCoralDetectedIndexer();
  }
}
