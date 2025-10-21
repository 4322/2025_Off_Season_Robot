package frc.robot.subsystems;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffector.EndEffectorStates;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.SingleTagCamera;
import frc.robot.util.ReefStatus;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public static final Timer startTimer = new Timer();
  private boolean requestIdle = false;
  private boolean requestHomed = false;
  private boolean requestEject = false;
  private boolean requestMeatballPrescore = false;
  private boolean requestMeatballScore = false;
  private boolean requestIntakeMeatballFloor = false;
  private boolean requestDescoreMeatball = false;
  private boolean requestPrescoreRigatoni = false;
  private boolean requestScoreRigatoni = false;
  private boolean requestPreClimb = false;
  private boolean requestClimb = false;
  private boolean ishomed = false;
  private Timer rigatoniPickupTimer = new Timer();
  private boolean rigatoniElevatorPickUp = false;
  private boolean scoreBackSide = false;

  public enum Superstates {
    HOMELESS,
    DISABLED,
    IDLE,
    EJECT,
    MEATBALL_IDLE,
    MEATBALL_PRESCORE,
    MEATBALL_SCORE,
    INTAKE_MEATBALL_FLOOR,
    DESCORE_MEATBALL,
    END_EFFECTOR_RIGATONI_PICKUP,
    RIGATONI_HELD,
    PRESCORE_RIGATONI,
    SCORE_RIGATONI,
    SAFE_SCORE_MEATBALL_RETRACT,
    PRECLIMB,
    CLIMB
  }

  public static enum Level {
    L1,
    L2,
    L3,
    L4
  }

  Level level = Level.L1;

  public static enum OperationMode {
    TeleAUTO,
    MANUAL,
  }

  OperationMode mode = OperationMode.TeleAUTO;

  Superstates state = Superstates.HOMELESS;
  Superstates prevState = Superstates.HOMELESS;

  private EndEffector endEffector;
  private Arm arm;
  private Elevator elevator;
  private IntakeSuperstructure intakeSuperstructure;
  private Vision vision;

  // Add this variable to track the previous state of the home button

  public Superstructure(
      EndEffector endEffector,
      Arm arm,
      Elevator elevator,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    this.endEffector = endEffector;
    this.arm = arm;
    this.elevator = elevator;
    this.intakeSuperstructure = intakeSuperstructure;
    this.vision = vision;
  }

  @Override
  public void periodic() {

    if (DriverStation.isDisabled() && ishomed) {
      state = Superstates.DISABLED;
    }

    // The home button can only be activated when the robot is disabled, so accept it from any state
    if (requestHomed) {
      elevator.setHomePosition();
      arm.setHomePosition();
      intakeSuperstructure.setHome();
      requestHomed = false;
      ishomed = true;
      state = Superstates.DISABLED;
    }

    Logger.recordOutput("Superstructure/currentState", state.toString());
    Logger.recordOutput("Superstructure/currentAutoState", mode.toString());
    Logger.recordOutput("Superstructure/requestedLevel", level);

    if (requestEject) {
      state = Superstates.EJECT;
    }

    prevState = state;
    switch (state) {
      case HOMELESS:
        elevator.reset();
        arm.reset();
        break;

      case DISABLED:
        elevator.reset();
        arm.reset();
        if (DriverStation.isEnabled()) {
          state = Superstates.IDLE;
          elevator.reset();
          arm.reset();
        }
        break;
      case IDLE:
        if (requestEject) {
          state = Superstates.EJECT;
        } else if (intakeSuperstructure.isRigatoniDetectedPickupArea()) {
          endEffector.idle();
          elevator.idle();
          arm.idle();
          if (arm.atSetpoint() && elevator.atSetpoint()) {
            state = Superstates.END_EFFECTOR_RIGATONI_PICKUP;
          }
        } else if (requestIntakeMeatballFloor) {
          state = Superstates.INTAKE_MEATBALL_FLOOR;
        } else if (requestDescoreMeatball) {
          state = Superstates.DESCORE_MEATBALL;
        } else if (requestPreClimb && DriverStation.getMatchTime() < 30.0) {
          state = Superstates.PRECLIMB;
        } else if (endEffector.hasRigatoni()) {
          state = Superstates.RIGATONI_HELD;
        } else {
          endEffector.idle();
          elevator.idle();
          arm.idle();
        }

        break;
      case EJECT:
        elevator.eject();
        arm.eject();
        endEffector.eject();

        if (requestIdle && !endEffector.hasMeatball() && !endEffector.hasRigatoni()) {
          state = Superstates.IDLE;
        } else if (requestIdle && endEffector.hasMeatball()) {
          state = Superstates.MEATBALL_IDLE;
        } else if (requestIdle && endEffector.hasRigatoni()) {
          state = Superstates.RIGATONI_HELD;
        }

        break;
      case MEATBALL_IDLE:
        endEffector.holdMeatball();
        arm.meatballHold();
        elevator.meatballHold();
        if (requestEject) {
          state = Superstates.EJECT;
        } else if (!endEffector.hasMeatball()) {
          state = Superstates.IDLE;
        } else if (requestMeatballPrescore) {
          state = Superstates.MEATBALL_PRESCORE;
        }

        break;
      case MEATBALL_PRESCORE:
        elevator.scoreMeatball();
        if (elevator.atSetpoint()) {
          arm.scoreMeatball(scoreBackSide);
        }

        if (requestIdle) {
          if (elevator.getElevatorHeightMeters()
              >= Constants.Elevator.safeBargeRetractHeightMeters) {
            state = Superstates.SAFE_SCORE_MEATBALL_RETRACT;
          } else {
            state = Superstates.MEATBALL_IDLE;
          }
        } else if (requestMeatballScore && arm.atSetpoint()) {
          state = Superstates.MEATBALL_SCORE;
        }

        break;
      case MEATBALL_SCORE:
        endEffector.releaseMeatball();
        if (requestIdle) {
          state = Superstates.SAFE_SCORE_MEATBALL_RETRACT;
        }

        break;
      case INTAKE_MEATBALL_FLOOR: // Needs to move up then arm out then back down
        elevator.meatballGround();
        arm.meatballGround();
        endEffector.intakeMeatball();

        if (requestIdle) {
          if (endEffector.hasMeatball()) {
            state = Superstates.MEATBALL_IDLE;
          } else {
            state = Superstates.IDLE;
          }
        }

        break;
      case DESCORE_MEATBALL:
        arm.meatballReef();
        elevator.meatballReef(level);
        endEffector.intakeMeatball();

        if (requestIdle) {
          if (endEffector.hasMeatball()) {
            state = Superstates.MEATBALL_IDLE;
          } else if (!endEffector.hasMeatball()) {
            state = Superstates.IDLE;
          }
        }
        break;
      case END_EFFECTOR_RIGATONI_PICKUP:
        Logger.recordOutput("Superstructure/RigatoniStopIsDisabled", rigatoniElevatorPickUp);
        arm.idle();
        endEffector.intakeRigatoni();

        if (!rigatoniElevatorPickUp) {
          rigatoniPickupTimer.start();
        }

        if (rigatoniPickupTimer.hasElapsed(Constants.EndEffector.rigatoniGrabDelaySeconds)) {
          elevator.pickupRigatoni();
          rigatoniElevatorPickUp = true;
        }

        if (rigatoniPickupTimer.hasElapsed(Constants.Elevator.rigatoniDetectionHeightThresholdSecs)) {
          if (endEffector.hasRigatoni()) {
            rigatoniElevatorPickUp = false;
            rigatoniPickupTimer.stop();
            rigatoniPickupTimer.reset();
            state = Superstates.RIGATONI_HELD;
            rigatoniElevatorPickUp = false;
          } else if (!endEffector.hasRigatoni()
              && !intakeSuperstructure.isRigatoniDetectedPickupArea()
              && arm.atSetpoint()
              && getElevatorHeight() >= Constants.Elevator.minElevatorSafeHeightMeters) {
            rigatoniElevatorPickUp = false;
            rigatoniPickupTimer.stop();
            rigatoniPickupTimer.reset();
            state = Superstates.IDLE;
          }
        }

        break;
      case RIGATONI_HELD:
        arm.rigatoniHold();
        elevator.rigatoniHold();
        endEffector.holdRigatoni();

        if (requestEject) {
          state = Superstates.EJECT;
        } else if (!endEffector.hasRigatoni()) {
          state = Superstates.IDLE;
        } else if (requestPrescoreRigatoni) {
          state = Superstates.PRESCORE_RIGATONI;
        }
        break;
      case PRESCORE_RIGATONI:
        arm.prescoreRigatoni(level);
        elevator.prescoreRigatoni(level);
        if (requestScoreRigatoni && arm.atSetpoint() && elevator.atSetpoint()) {
          state = Superstates.SCORE_RIGATONI;
        } else if (requestIdle) {
          if (endEffector.hasRigatoni()) {
            state = Superstates.RIGATONI_HELD;
          } else {
            state = Superstates.IDLE;
          }
        }

        break;
      case SCORE_RIGATONI:
        arm.scoreRigatoni(level);
        elevator.scoreRigatoni(level);
        if (level == Level.L1) {
          if (arm.atSetpoint() && elevator.atSetpoint()) {
            endEffector.releaseRigatoniL1();
          }
        } else if ((arm.getAngleDegrees() <= arm.scoreReleaseSetpoint())) {
          endEffector.releaseRigatoniNormal();
        } else {
          if (elevator.atSetpoint() && arm.atSetpoint()) {
            endEffector.releaseRigatoniNormal();
          }
        }

        if (requestIdle) {
          if (endEffector.hasRigatoni()) {
            state = Superstates.RIGATONI_HELD;
          } else {
            state = Superstates.IDLE;
          }
        } else if (requestDescoreMeatball && !endEffector.hasRigatoni()) {
          state = Superstates.DESCORE_MEATBALL;
        }
        break;
      case SAFE_SCORE_MEATBALL_RETRACT:
        arm.safeBargeRetract();
        if (arm.atSetpoint()) {
          elevator.safeBargeRetract();
          if (elevator.atSetpoint()) {
            if (!endEffector.hasMeatball()) {
              state = Superstates.IDLE;
            } else if (endEffector.hasMeatball()) {
              state = Superstates.MEATBALL_IDLE;
            }
          }
        }

        break;
      case PRECLIMB:

        // TODO
        break;
      case CLIMB:
        // TODO
        break;
    }
  }

  private void unsetAllRequests() {
    // don't clear requestHomed since it must be processed
    requestEject = false;
    requestIdle = false;
    requestMeatballScore = false;
    requestIntakeMeatballFloor = false;
    requestDescoreMeatball = false;
    requestMeatballPrescore = false;
    requestPrescoreRigatoni = false;
    requestScoreRigatoni = false;
    requestPreClimb = false;
    requestClimb = false;
    requestIntakeMeatballFloor = false;
  }

  public void requestOperationMode(OperationMode mode) {
    this.mode = mode;
  }

  public boolean armAtSetpoint() {
    return arm.atSetpoint();
  }

  public boolean elevatorAtSetpoint() {
    return elevator.atSetpoint();
  }

  public boolean isAutoOperationMode() {
    return mode == OperationMode.TeleAUTO;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void requestEject() {
    unsetAllRequests();
    requestEject = true;
  }

  public void requestMeatballPrescore(boolean prescoreBackSide) {
    unsetAllRequests();
    requestMeatballPrescore = true;
    this.scoreBackSide = prescoreBackSide;
  }

  public void requestMeatballScore() {
    unsetAllRequests();
    requestMeatballScore = true;
  }

  public void requestIntakeMeatballFloor() {
    unsetAllRequests();
    requestIntakeMeatballFloor = true;
  }

  public void requestDescoreMeatball(Level level) {
    unsetAllRequests();
    requestDescoreMeatball = true;
    this.level = level;
  }

  public void requestPrescoreRigatoni(Level level) {
    unsetAllRequests();
    requestPrescoreRigatoni = true;
    this.level = level;
  }

  public void requestScoreRigatoni(Level level) {
    unsetAllRequests();
    requestScoreRigatoni = true;
    this.level = level;
  }

  public void requestPreClimb() {
    unsetAllRequests();
    requestPreClimb = true;
  }

  public void requestClimb() {
    unsetAllRequests();
    requestClimb = true;
  }

  public void requestUnhome() {
    endEffector.idle(); // let any meatball pop-out, rigatoni must have already been ejected
    unsetAllRequests();
    state = Superstates.HOMELESS;
  }

  public void requestReHome() {
    ishomed = true;
    elevator.setReHome();
    arm.setReHome();
    intakeSuperstructure.setReHome();
    state = Superstates.IDLE;
  }

  public boolean isRigatoniHeld() {
    return endEffector.hasRigatoni();
  }

  public boolean isMeatballHeld() {
    return endEffector.hasMeatball();
  }

  public double getElevatorHeight() {
    return elevator.getElevatorHeightMeters();
  }

  public double getArmAngle() {
    return arm.getAngleDegrees();
  }

  public EndEffectorStates getEndEffectorState() {
    return endEffector.getState();
  }

  // Return the state executed in the current periodic cycle
  public Superstates getState() {
    return prevState;
  }

  public ReefStatus getReefStatus() {
    return vision.getReefStatus();
  }

  public IntakeSuperstructure getIntakeSuperstructure() {
    return intakeSuperstructure;
  }

  public void homeButtonActivated() {
    requestHomed = true;
  }

  public void CoastMotors() {
    arm.stop();
    elevator.stop(IdleMode.kCoast);
    intakeSuperstructure.deployer.stop(IdleMode.kCoast);
  }

  public void BreakMotors() {
    arm.stop();
    elevator.stop(IdleMode.kBrake);
    intakeSuperstructure.deployer.stop(IdleMode.kBrake);
  }

  public void enableGlobalPose() {
    vision.enableGlobalPose();
  }

  public void enableSingleTag(int tagID, SingleTagCamera cameraToUse) {
    vision.enableSingleTagSingleCam(tagID, cameraToUse);
  }
}
