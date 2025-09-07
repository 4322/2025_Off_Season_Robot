package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public static final Timer startTimer = new Timer();
  private boolean requestHomed = false;
  private boolean cancelEject = false;
  private boolean requestEject = false;
  private boolean requestAlgaePrescore = false;
  private boolean cancelAlgaePrescore = false;
  private boolean requestAlgaeScore = false;
  private boolean requestIntakeAlgaeFloor = false;
  private boolean requestDescoreAlgae = false;
  private boolean requestEndEffectorCoralPickup = false;
  private boolean requestCoralHeld = false;
  private boolean requestPrescoreCoral = false;
  private boolean cancelPrescoreCoral = false;
  private boolean requestScoreCoral = false;
  private boolean requestSafeScoreAlgaeRetract = false;
  private boolean requestPreClimb = false;
  private boolean requestClimb = false;
  private boolean requestswitchOperationMode = false;

  public enum Superstates {
    UNHOMED,
    IDLE,
    EJECT,
    ALGAE_IDLE,
    ALGAE_PRESCORE,
    ALGAE_SCORE,
    INTAKE_ALGAE_FLOOR,
    DESCORE_ALGAE,
    END_EFFECTOR_CORAL_PICKUP,
    CORAL_HELD,
    PRESCORE_CORAL,
    SCORE_CORAL,
    SAFE_SCORE_ALGAE_RETRACT,
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
    Auto,
    Manual,
  }

  OperationMode mode = OperationMode.Auto;

  Superstates state = Superstates.UNHOMED;
  Superstates prevState = Superstates.UNHOMED;
  Superstates savedState = Superstates.UNHOMED;

  private EndEffector endEffector;
  private Arm arm;
  private Indexer indexer;
  private Elevator elevator;
  private Drive drive;
  private Vision vision;
  private IntakeSuperstructure intakeSuperstructure;

  // Add this variable to track the previous state of the home button

  public Superstructure(
      EndEffector endEffector,
      Arm arm,
      Indexer indexer,
      Elevator elevator,
      Drive drive,
      Vision vision,
      IntakeSuperstructure intakeSuperstructure) {
    this.endEffector = endEffector;
    this.arm = arm;
    this.elevator = elevator;
    this.drive = drive;
    this.indexer = indexer;
    this.vision = vision;
    this.intakeSuperstructure = intakeSuperstructure;
  }

  @Override
  public void periodic() {

    // The home button can only be activated when the robot is disabled, so accept it from any state
    if (requestHomed) {
      elevator.setHomePosition();
      arm.setHomePosition();
      requestHomed = false;
      state = Superstates.IDLE;
    }

    Logger.recordOutput("Superstructure/currentState", state.toString());

    switch (state) {
      case UNHOMED:
        break;
      case IDLE:
        endEffector.idle();
        elevator.idle();
        arm.idle();

        if (requestEject) {
          state = Superstates.EJECT;
        } else if (indexer.isCoralDetectedPickupArea()
            && arm.atSetpoint()
            && elevator.atSetpoint()) {
          state = Superstates.END_EFFECTOR_CORAL_PICKUP;
        } else if (requestIntakeAlgaeFloor && !endEffector.hasAlgae()) {
          state = Superstates.INTAKE_ALGAE_FLOOR;
        } else if (requestDescoreAlgae) {
          state = Superstates.DESCORE_ALGAE;
        } else if (requestPreClimb && DriverStation.getMatchTime() < 30.0) {
          state = Superstates.PRECLIMB;
        }

        break;
      case EJECT:
        elevator.eject();
        arm.eject();
        endEffector.eject();

        if (!requestEject && !endEffector.hasAlgae() && !endEffector.hasCoral()) {
          state = Superstates.IDLE;
        } else if (!requestEject && endEffector.hasAlgae()) {
          state = Superstates.ALGAE_IDLE;
        } else if (!requestEject && endEffector.hasCoral()) {
          state = Superstates.CORAL_HELD;
        }

        break;
      case ALGAE_IDLE:
        arm.algaeHold();
        endEffector.holdAlgae();
        if (requestEject) {
          state = Superstates.EJECT;
        } else if (!endEffector.hasAlgae()) {
          state = Superstates.IDLE;
        } else if (requestAlgaePrescore) {
          state = Superstates.ALGAE_PRESCORE;
        }

        break;
      case ALGAE_PRESCORE:
        arm.scoreAlgae();
        elevator.scoreAlgae();

        if (!requestAlgaePrescore) {
          state = Superstates.ALGAE_IDLE;
        } else if (requestAlgaeScore && arm.atSetpoint() && elevator.atSetpoint()) {
          state = Superstates.ALGAE_SCORE;
        }

        break;
      case ALGAE_SCORE:
        endEffector.releaseAlgae();

        if (!endEffector.hasAlgae() || !requestAlgaePrescore) {
          state = Superstates.SAFE_SCORE_ALGAE_RETRACT;
        } else if (endEffector.hasAlgae()) {
          state = Superstates.ALGAE_IDLE;
        }

        break;
      case INTAKE_ALGAE_FLOOR: // Needs to move up then arm out then back down
        elevator.algaeGround();
        arm.algaeGround();
        endEffector.intakeAlgae();

        if (endEffector.hasAlgae()) {
          state = Superstates.ALGAE_IDLE;
        } else if (!requestIntakeAlgaeFloor) {
          state = Superstates.IDLE;
        }

        break;
      case DESCORE_ALGAE:
        arm.algaeReef();
        elevator.algaeReef(level);
        endEffector.intakeAlgae();

        if (endEffector
            .hasAlgae() /*&& atSafeDrive (Place Holder: Will be delt with in commands)*/) {
          state = Superstates.ALGAE_IDLE;
        } else if (!requestDescoreAlgae && !endEffector.hasAlgae()) {
          state = Superstates.IDLE;
        }

        break;
      case END_EFFECTOR_CORAL_PICKUP:
        if (indexer.isCoralDetectedPickupArea()) {
          elevator.pickupCoral();
          endEffector.intakeCoral();
        }

        if (endEffector.hasCoral() && elevator.atSetpoint()) {
          state = Superstates.CORAL_HELD;
        } else if (!endEffector.hasCoral()
            && !indexer.isCoralDetectedPickupArea()
            && elevator.atSetpoint()) {
          state = Superstates.IDLE;
        }

        break;
      case CORAL_HELD:
        arm.coralHold();
        elevator.coralHold();

        if (requestEject) {
          state = Superstates.EJECT;
        } else if (!endEffector.hasCoral()) {
          state = Superstates.IDLE;
        } else if (requestPrescoreCoral) {
          state = Superstates.PRESCORE_CORAL;
        }
        break;
      case PRESCORE_CORAL:
        arm.prescoreCoral(level);
        elevator.prescoreCoral(level);

        if (requestScoreCoral && arm.atSetpoint() && elevator.atSetpoint()) {
          state = Superstates.SCORE_CORAL;
        } else if (!requestPrescoreCoral && endEffector.hasCoral()) {
          state = Superstates.CORAL_HELD;
        }
        break;
      case SCORE_CORAL:
        arm.scoreCoral(level);
        elevator.scoreCoral(level);
        endEffector.releaseCoral();

        if (!endEffector.hasCoral()) {
          state = Superstates.IDLE;
        } else if (endEffector.hasCoral()) {
          state = Superstates.CORAL_HELD;
        }
        break;
      case SAFE_SCORE_ALGAE_RETRACT:
        endEffector.idle();
        arm.safeBargeRetract();
        elevator.safeBargeRetract();

        if (!endEffector.hasAlgae() || !requestAlgaePrescore) {
          state = Superstates.IDLE;
        } else if (endEffector.hasAlgae()) {
          state = Superstates.ALGAE_IDLE;
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
    requestAlgaeScore = false;
    requestIntakeAlgaeFloor = false;
    requestDescoreAlgae = false;
    requestEndEffectorCoralPickup = false;
    requestCoralHeld = false;
    requestPrescoreCoral = false;
    requestScoreCoral = false;
    requestSafeScoreAlgaeRetract = false;
    requestPreClimb = false;
    requestClimb = false;
    requestswitchOperationMode = false;
    requestIntakeAlgaeFloor = false;
  }

  public void requestOperationMode(OperationMode mode) {
    unsetAllRequests();
    requestswitchOperationMode = true;
  }

  public boolean isAutoOperationMode() {
    if (mode == OperationMode.Auto) {
      return true;
    } else {
      return false;
    }
  }

  public void requestEject() {
    unsetAllRequests();
    requestEject = true;
  }

  public void cancelEject() {
    unsetAllRequests();
    requestEject = false;
  }

  public void requestAlgaePrescore() {
    unsetAllRequests();
    requestAlgaePrescore = true;
  }

  public void cancelAlgaePrescore() {
    unsetAllRequests();
    requestAlgaePrescore = false;
  }

  public void requestAlgaeScore() {
    unsetAllRequests();
    requestAlgaeScore = true;
  }

  public void requestIntakeAlgaeFloor() {
    unsetAllRequests();
    requestIntakeAlgaeFloor = true;
  }

  public void cancelIntakeAlgaeFloor() {
    unsetAllRequests();
    requestIntakeAlgaeFloor = false;
  }

  public void requestIntakeAlgaeReef(Level level) {
    unsetAllRequests();
    requestIntakeAlgaeFloor = true;
  }

  public void requestDescoreAlgae(Level level) {
    unsetAllRequests();
    requestDescoreAlgae = true;
  }

  public void cancelDescoreAlgae() {
    unsetAllRequests();
    requestDescoreAlgae = false;
  }

  public void requestEndEffectorCoralPickup() {
    unsetAllRequests();
    requestEndEffectorCoralPickup = true;
  }

  public void requestCoralHeld() {
    unsetAllRequests();
    requestCoralHeld = true;
  }

  public void requestPrescoreCoral(Level coralLevel) {
    unsetAllRequests();
    requestPrescoreCoral = true;
  }

  public void cancelPrescoreCoral() {
    unsetAllRequests();
    requestPrescoreCoral = false;
  }

  public void requestScoreCoral(Level coralLevel) {
    unsetAllRequests();
    requestScoreCoral = true;
  }

  public void requestSafeScoreAlgaeRetract() {
    unsetAllRequests();
    requestSafeScoreAlgaeRetract = true;
  }

  public void requestPreClimb() {
    unsetAllRequests();
    requestPreClimb = true;
  }

  public void requestClimb() {
    unsetAllRequests();
    requestClimb = true;
  }

  public boolean isCoralHeld() {
    return endEffector.hasCoral();
  }

  public boolean isAlgaeHeld() {
    return endEffector.hasAlgae();
  }

  public double getElevatorHeight() {
    return elevator.getElevatorHeightMeters();
  }

  public double getArmAngle() {
    return arm.getAngleDegrees();
  }

  public Superstates getState() {
    return state;
  }

  public Superstates getPrevState() {
    return prevState;
  }

  public void getReefStatus() {
    // TODO when get vision working
    // return visionPoseEstimation.getReefStatus()
  }

  public IntakeSuperstructure getIntakeSuperstructure() {
    return intakeSuperstructure;
  }

  public void homeButtonActivated() {
    requestHomed = true;
  }

  // Other Methods are related to Vision Pose Estimation
}
