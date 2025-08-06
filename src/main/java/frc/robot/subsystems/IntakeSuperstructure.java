package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.deployer.Deployer;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.rollers.Rollers;


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

    public IntakeSuperstructure(Deployer deployer, Rollers rollers, Indexer indexer) {
        this.deployer = deployer;
        this.rollers = rollers;
        this.indexer = indexer;
    }

    @Override
    public void periodic() {
        switch(state) {
            case START:
                if (requestRetractIdle) {
                    state = IntakeSuperstates.RETRACT_IDLE;
                }
            break;
            case RETRACT_IDLE:
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
                 if (rollers.isCoralDetected() && retractLockedOutState == RetractLockedOutStates.FALSE) {
                    retractLockedOutState = RetractLockedOutStates.INDEXER;
                    retractTimeOutIndexerTimer.reset();
                    retractTimeOutIndexerTimer.start();
                } else if (isCoralDetectedIndexer() && retractLockedOutState == RetractLockedOutStates.INDEXER) {
                    retractLockedOutState = RetractLockedOutStates.PICKUP;
                    retractTimeOutIndexerTimer.stop();
                    retractTimeOutPickupAreaTimer.reset();
                    retractTimeOutPickupAreaTimer.start();
                } else if ((isCoralDetectedPickupArea() || retractTimeOutPickupAreaTimer.hasElapsed(Constants.IntakeSuperstructure.RETRACT_TIMEOUT_SECONDS)) && retractLockedOutState == RetractLockedOutStates.PICKUP) {
                    retractLockedOutState = RetractLockedOutStates.FALSE;
                }
                if (requestRetractIdle && retractLockedOutState == RetractLockedOutStates.FALSE) {
                    state = IntakeSuperstates.RETRACT_IDLE;
                }
                if (isCoralDetectedIndexer() || isCoralDetectedPickupArea()) {
                    state = IntakeSuperstates.REJECT;
                }
                if (requestIntakeEject) {
                    state = IntakeSuperstates.INTAKE_EJECT;
                }
            break;
            case REJECT:

            break;
            case INTAKE_EJECT:

            break;
        }
    }
    private void unsetAllRequests() {
        requestRetractIdle = false;
        requestFeed = false;
        requestReject = false;
        requestIntakeEject = false;
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

