package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DrivePanrStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.deployer.Deployer;
import frc.robot.subsystems.tongs.Tongs;
import frc.robot.subsystems.pastaWheels.PastaWheels;
import frc.robot.subsystems.rollingPins.RollingPins;
import org.littletonrobotics.junction.Logger;

public class IntakeSuperstructure extends SubsystemBase {

  private boolean requestHomed;
  private boolean requestRetractIdle;
  private boolean requestDeploy;
  private boolean requestIntakeEject;
  private boolean isHomed = false;
  private boolean requestPastaWheelsEject;

  private enum RetractLockedOutStates {
    FALSE,
    INDEXER,
    PICKUP,
  }

  private Timer retractTimeOutPastaWheelsTimer = new Timer();
  private Timer retractTimeOutPickupAreaTimer = new Timer();
  private RetractLockedOutStates retractLockedOutState = RetractLockedOutStates.FALSE;

  private IntakeSuperstates state = IntakeSuperstates.HOMELESS;

  public Deployer deployer;
  private RollingPins rollingPins;
  private PastaWheels pastaWheels;

  public static enum IntakeSuperstates {
    HOMELESS,
    DISABLED,
    RETRACT_IDLE,
    FEED,
    SLOW_REJECT,
    INTAKE_EJECT,
    INDEXER_EJECT
  }

  public IntakeSuperstructure(
      Tongs tongs, Deployer deployer, RollingPins rollingPins, PastaWheels pastaWheels) {
    this.deployer = deployer;
    this.rollingPins = rollingPins;
    this.pastaWheels = pastaWheels;
  }

  @Override
  public void periodic() {

    // The home button can only be activated when the robot is disabled, so accept it from any state
    if (requestHomed) {
      deployer.setHome();
      requestHomed = false;
      isHomed = true;
      if (DrivePanrStation.isEnabled()) {
        state = IntakeSuperstates.RETRACT_IDLE;
      } else {
        state = IntakeSuperstates.DISABLED;
      }
    }

    if (DrivePanrStation.isDisabled() && isHomed) {
      state = IntakeSuperstates.DISABLED;
    }

    Logger.recordOutput("IntakeSuperstructure/State", state.toString());

    switch (state) {
      case HOMELESS:
        break;
      case DISABLED:
        if (DrivePanrStation.isEnabled()) {
          state = IntakeSuperstates.RETRACT_IDLE;
        }
        break;
      case RETRACT_IDLE:
        deployer.retract();
        if (isRigatoniDetectedPickupArea() || RobotContainer.getSuperstructure().isRigatoniHeld()) {
          rollingPins.rejectSlow();
          pastaWheels.feedSlow();
        } else {
          rollingPins.feedSlow();
          pastaWheels.feedSlow();
        }
        if (requestPastaWheelsEject) {
          state = IntakeSuperstates.INDEXER_EJECT;
        } else if (requestIntakeEject) {
          state = IntakeSuperstates.INTAKE_EJECT;
        } else if (requestDeploy) {
          if (isRigatoniDetectedPastaWheels()
              || isRigatoniDetectedPickupArea()
              || RobotContainer.getSuperstructure().isRigatoniHeld()) {
            state = IntakeSuperstates.SLOW_REJECT;
          } else {
            state = IntakeSuperstates.FEED;
          }
        }
        break;
      case FEED:
        rollingPins.feed();
        pastaWheels.feed();
        deployer.deploy();

        if (isRigatoniDetectedPickupArea() || RobotContainer.getSuperstructure().isRigatoniHeld()) {
          state = IntakeSuperstates.SLOW_REJECT;
        }
        if (requestPastaWheelsEject) {
          state = IntakeSuperstates.INDEXER_EJECT;
        }

        switch (retractLockedOutState) {
            // Default case, starts lockout timer when rigatoni is detected in rollingPins
          case FALSE:
            if (rollingPins.isRigatoniPickupDetected()) {
              retractLockedOutState = RetractLockedOutStates.INDEXER;
              retractTimeOutPastaWheelsTimer.stop();
              retractTimeOutPastaWheelsTimer.reset();
              retractTimeOutPastaWheelsTimer.start();
            }
            break;
          case INDEXER:
            // If rigatoni isn't detected in pastaWheels after x time, clear lockout; Otherwise start
            // pickup area timer
            if (isRigatoniDetectedPastaWheels()) {
              retractLockedOutState = RetractLockedOutStates.PICKUP;
              retractTimeOutPastaWheelsTimer.stop();
              retractTimeOutPickupAreaTimer.reset();
              retractTimeOutPickupAreaTimer.start();
            } else if (!isRigatoniDetectedPastaWheels()
                && retractTimeOutPastaWheelsTimer.hasElapsed(
                    Constants.IntakeSuperstructure.pastaWheelsRetractTimeoutSeconds)) {
              retractTimeOutPastaWheelsTimer.stop();
              retractLockedOutState = RetractLockedOutStates.FALSE;
            }
            break;
          case PICKUP:
            retractTimeOutPastaWheelsTimer.stop();
            retractTimeOutPastaWheelsTimer.reset();
            // If rigatoni is detected in pickup area/end effector or x time has passed, clear lockout
            if ((isRigatoniDetectedPickupArea() || RobotContainer.getSuperstructure().isRigatoniHeld())
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
        } else if (isRigatoniDetectedPickupArea()
            || RobotContainer.getSuperstructure().isRigatoniHeld()) {
          state = IntakeSuperstates.SLOW_REJECT;
        }

        break;
      case SLOW_REJECT:
        deployer.deploy();
        rollingPins.rejectSlow();
        pastaWheels.rejectSlow();

        if (requestIntakeEject) {
          state = IntakeSuperstates.INTAKE_EJECT;
        } else if (requestRetractIdle) {
          state = IntakeSuperstates.RETRACT_IDLE;
        } else if (!isRigatoniDetectedPastaWheels()
            && !isRigatoniDetectedPickupArea()
            && !RobotContainer.getSuperstructure().isRigatoniHeld()) {
          state = IntakeSuperstates.FEED;
        }

        break;
      case INTAKE_EJECT:
        deployer.eject();
        rollingPins.eject();
        pastaWheels.reject();
        if (requestRetractIdle) {
          state = IntakeSuperstates.RETRACT_IDLE;
        }
        break;
      case INDEXER_EJECT:
        deployer.deploy();
        rollingPins.eject();

        if (RobotContainer.drivePanr.rightBumper().getAsBoolean()) {
          pastaWheels.ejectRight();
        } else if (RobotContainer.drivePanr.leftBumper().getAsBoolean()) {
          pastaWheels.ejectLeft();
        } else {
          requestPastaWheelsEject = false;
        }
        if (!requestPastaWheelsEject) {
          if (requestRetractIdle) {
            state = IntakeSuperstates.RETRACT_IDLE;
          } else if (requestDeploy) {
            state = IntakeSuperstates.FEED;
          }
        }
        break;
    }
  }

  private void unsetAllRequests() {
    // don't clear requestHomed since it must be processed
    requestRetractIdle = false;
    requestIntakeEject = false;
    requestDeploy = false;
    requestPastaWheelsEject = false;
  }

  public IntakeSuperstates getState() {
    return state;
  }

  public void requestRetractIdle() {
    unsetAllRequests();
    requestRetractIdle = true;
  }

  public void requestIntake() {
    // Transitions to Feeding or Rejecting based on if rigatoni in robot
    unsetAllRequests();
    requestDeploy = true;
  }

  public void requestEject() {
    unsetAllRequests();
    requestIntakeEject = true;
  }

  public void requestIdexerEject() {
    requestPastaWheelsEject = true;
  }

  public void requestUnhome() {
    deployer.clearHome();
    rollingPins.idle();
    pastaWheels.idle();
    state = IntakeSuperstates.HOMELESS;
    unsetAllRequests();
  }

  public boolean isRigatoniDetectedPickupArea() {
    return pastaWheels.isRigatoniDetectedPickupArea();
  }

  public boolean isRigatoniDetectedPastaWheels() {
    return pastaWheels.isRigatoniDetectedPastaWheels();
  }

  public void setHome() {
    requestHomed = true;
  }

  public void setReHome() {
    deployer.setReHome();
    isHomed = true;
    if (DrivePanrStation.isEnabled()) {
      state = IntakeSuperstates.RETRACT_IDLE;
    } else {
      state = IntakeSuperstates.DISABLED;
    }
  }

  public boolean isRigatoniDetectedIntake() {
    return rollingPins.isRigatoniPickupDetected()
        || pastaWheels.isRigatoniDetectedPickupArea()
        || pastaWheels.isRigatoniDetectedPastaWheels();
  }

  public IntakeSuperstates getIntakeSuperstate() {
    return state;
  }
}
