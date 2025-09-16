package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.indexer.Indexer;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public static final Timer startTimer = new Timer();
  private boolean requestIdle = false;
  private boolean requestHomed = false;
  private boolean requestEject = false;
  private boolean requestAlgaePrescore = false;
  private boolean requestAlgaeScore = false;
  private boolean requestIntakeAlgaeFloor = false;
  private boolean requestDescoreAlgae = false;
  private boolean requestPrescoreCoral = false;
  private boolean requestScoreCoral = false;
  private boolean requestPreClimb = false;
  private boolean requestClimb = false;

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
    AUTO,
    MANUAL,
  }

  OperationMode mode = OperationMode.AUTO;

  Superstates state = Superstates.UNHOMED;

  private EndEffector endEffector;
  private Arm arm;
  private Indexer indexer;
  private Elevator elevator;
  private IntakeSuperstructure intakeSuperstructure;

  // Add this variable to track the previous state of the home button

  public Superstructure(
      EndEffector endEffector,
      Arm arm,
      Indexer indexer,
      Elevator elevator,
      IntakeSuperstructure intakeSuperstructure) {
    this.endEffector = endEffector;
    this.arm = arm;
    this.elevator = elevator;
    this.indexer = indexer;
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
    Logger.recordOutput("Superstructure/requestedLevel", level);

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
        } else if (requestAlgaePrescore && arm.atSetpoint() && elevator.atSetpoint()) {
          state = Superstates.ALGAE_PRESCORE;
        }

        break;
      case ALGAE_PRESCORE:
        elevator.scoreAlgae();
        if (elevator.atSetpoint()) {
          arm.scoreAlgae();
        }

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

        if (endEffector.hasAlgae() && requestIdle) {
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
        endEffector.holdCoral();

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
        } else if (requestIdle && endEffector.hasCoral()) {
          state = Superstates.CORAL_HELD;
        }
        break;
      case SCORE_CORAL:
        arm.scoreCoral(level);
        elevator.scoreCoral(level);
        endEffector.releaseCoral();

        if (!endEffector.hasCoral() && arm.atSetpoint() && elevator.atSetpoint()) {
          state = Superstates.IDLE;

        } else if (endEffector.hasCoral() && arm.atSetpoint() && elevator.atSetpoint()) {
          state = Superstates.CORAL_HELD;
        }
        break;
      case SAFE_SCORE_ALGAE_RETRACT:
        endEffector.idle();

        if (elevator.getElevatorHeightMeters() < Constants.Elevator.safeBargeRetractHeightMeters) {
          if (endEffector.hasAlgae()) {
            state = Superstates.ALGAE_IDLE;
          } else {
            state = Superstates.IDLE;
          }
        } else {
          arm.safeBargeRetract();
          if (arm.atSetpoint()) {
            elevator.safeBargeRetract();
            if (elevator.atSetpoint()) {
              if (endEffector.hasAlgae()) {
                state = Superstates.ALGAE_IDLE;
              } else {
                state = Superstates.IDLE;
              }
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
    requestAlgaeScore = false;
    requestIntakeAlgaeFloor = false;
    requestDescoreAlgae = false;
    requestPrescoreCoral = false;
    requestScoreCoral = false;
    requestPreClimb = false;
    requestClimb = false;
    requestIntakeAlgaeFloor = false;
  }

  public void requestOperationMode(OperationMode mode) {
    this.mode = mode;
  }

  public boolean isAutoOperationMode() {
    return mode == OperationMode.AUTO;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void requestEject() {
    unsetAllRequests();
    requestEject = true;
  }

  public void requestAlgaePrescore() {
    unsetAllRequests();
    requestAlgaePrescore = true;
  }

  public void requestAlgaeScore() {
    unsetAllRequests();
    requestAlgaeScore = true;
  }

  public void requestIntakeAlgaeFloor() {
    unsetAllRequests();
    requestIntakeAlgaeFloor = true;
  }

  public void requestDescoreAlgae(Level level) {
    unsetAllRequests();
    requestDescoreAlgae = true;
    this.level = level;
  }

  public void requestPrescoreCoral(Level level) {
    unsetAllRequests();
    requestPrescoreCoral = true;
    this.level = level;
  }

  public void requestScoreCoral(Level level) {
    unsetAllRequests();
    requestScoreCoral = true;
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
