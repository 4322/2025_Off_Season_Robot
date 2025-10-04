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
  private boolean requestDeploy;
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
        if (isCoralDetectedPickupArea() || RobotContainer.getSuperstructure().isCoralHeld() || RobotContainer.getSuperstructure().getState() == Superstructure.Superstates.END_EFFECTOR_CORAL_PICKUP) {
          rollers.rejectSlow();
          indexer.rejectSlow();
        } else {
          rollers.feedSlow();
          indexer.feedSlow();
        }
        if (requestIntakeEject) {
          state = IntakeSuperstates.INTAKE_EJECT;
        } else if (requestDeploy) {
          if (isCoralDetectedIndexer()
              || isCoralDetectedPickupArea()
              || RobotContainer.getSuperstructure().isCoralHeld()) {
            state = IntakeSuperstates.SLOW_REJECT;
          } else {
            state = IntakeSuperstates.FEED;
          }
        }
        break;
      case FEED:
        rollers.feed();
        indexer.feed();
        deployer.deploy();

        switch (retractLockedOutState) {
            // Default case, starts lockout timer when coral is detected in rollers
          case FALSE:
            if (rollers.isCoralPickupDetected()) {
              retractLockedOutState = RetractLockedOutStates.INDEXER;
              retractTimeOutIndexerTimer.stop();
              retractTimeOutIndexerTimer.reset();
              retractTimeOutIndexerTimer.start();
            }
            break;
          case INDEXER:
            // If coral isn't detected in indexer after x time, clear lockout; Otherwise start
            // pickup area timer
            if (isCoralDetectedIndexer()) {
              retractLockedOutState = RetractLockedOutStates.PICKUP;
              retractTimeOutIndexerTimer.stop();
              retractTimeOutPickupAreaTimer.reset();
              retractTimeOutPickupAreaTimer.start();
            } else if (!isCoralDetectedIndexer()
                && retractTimeOutIndexerTimer.hasElapsed(
                    Constants.IntakeSuperstructure.indexerRetractTimeoutSeconds)) {
              retractTimeOutIndexerTimer.stop();
              retractLockedOutState = RetractLockedOutStates.FALSE;
            }
            break;
          case PICKUP:
            retractTimeOutIndexerTimer.stop();
            retractTimeOutIndexerTimer.reset();
            // If coral is detected in pickup area/end effector or x time has passed, clear lockout
            if ((isCoralDetectedPickupArea() || RobotContainer.getSuperstructure().isCoralHeld())
                || retractTimeOutPickupAreaTimer.hasElapsed(
                    Constants.IntakeSuperstructure.pickupAreaRetractTimeoutSeconds)) {
              retractTimeOutPickupAreaTimer.stop();
              retractLockedOutState = RetractLockedOutStates.FALSE;
            }
            break;
        }

        if (requestIntakeEject) {
          state = IntakeSuperstates.INTAKE_EJECT;
        } else if (requestRetractIdle && retractLockedOutState == RetractLockedOutStates.FALSE) {
          state = IntakeSuperstates.RETRACT_IDLE;
        } else if (isCoralDetectedPickupArea()
            || RobotContainer.getSuperstructure().isCoralHeld()) {
          state = IntakeSuperstates.SLOW_REJECT;
        }

        break;
      case SLOW_REJECT:
        deployer.deploy();
        rollers.reject();
        indexer.feedSlow();

        if (requestIntakeEject) {
          state = IntakeSuperstates.INTAKE_EJECT;
        } else if (requestRetractIdle) {
          state = IntakeSuperstates.RETRACT_IDLE;
        } else if (!isCoralDetectedIndexer()
            && !isCoralDetectedPickupArea()
            && !RobotContainer.getSuperstructure().isCoralHeld()) {
          state = IntakeSuperstates.FEED;
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
    requestIntakeEject = false;
    requestDeploy = false;
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
    requestDeploy = true;
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

  public void setHome() {
    requestHomed = true;
  }

  public boolean isCoralDetectedIntake() {
    return rollers.isCoralPickupDetected()
        || indexer.isCoralDetectedPickupArea()
        || indexer.isCoralDetectedIndexer();
  }

  public IntakeSuperstates getIntakeSuperstate() {
    return state;
  }
}
  // TODO add manual homing procedure
// Review logic
