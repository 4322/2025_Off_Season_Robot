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

import frc.robot.subsystems.arm.ArmIONitrate;


public class Superstructure extends SubsystemBase {
  public static final Timer startTimer = new Timer();
  private boolean requestIdle = false;
  private boolean requestEject = false;
  private boolean requestAlgaeIdle = false;
  private boolean requestAlgaePrescore = false;
  private boolean requestAlgaeScore = false;
  private boolean requestIntakeAlgaeFloor = false;
  private boolean requestDescoreAlgae = false;
  private boolean requestEndEffectorCoralPickup = false;
  private boolean requestCoralHeld = false;
  private boolean requestPrescoreCoral = false;
  private boolean requestScoreCoral = false;
  private boolean requestSafeScoreAlgaeRetract = false;
  private boolean requestPreClimb = false;
  private boolean requestClimb = false;
  private boolean requestswitchOperationMode = false;

  private Level coralLevel = Level.L1;
  private Level algaeLevel = Level.L1;

  public enum Superstates {
    START,
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

  public static enum OperationMode {
    Auto,
    Manual,
  }

  OperationMode mode = OperationMode.Auto;

  Superstates state = Superstates.START;

  private EndEffector endEffector;
  private Arm arm;
  private Indexer indexer;
  private Elevator elevator;
  // TODO wait for Ellie to merge this into main: private Climber climber;
  private Drive drive;
  private Vision vision;
  private IntakeSuperstructure intakeSuperstructure;

  public Superstructure(
      EndEffector endEffector,
      Arm arm,
      Indexer indexer,
      Elevator elevator,
      // TODO wait for Ellie to merge this into main: Climber climber,
      Drive drive,
      Vision vision,
      IntakeSuperstructure intakeSuperstructure) {
    this.endEffector = endEffector;
    this.arm = arm;
    this.elevator = elevator;
    // TODO wait for Ellie to merge this into main: this.climber = climber;
    this.drive = drive;
    this.indexer = indexer;
    this.vision = vision;
    this.intakeSuperstructure = intakeSuperstructure;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/currentState", state.toString());

    switch (state) {
      case START: // TODO
        if (isHomeButtonPressed()) {
          if (startTimer.hasElapsed(1)) {
            elevator.setManualInitialization();
            arm.setManualInitialization();
          }
        }
        if (DriverStation.isEnabled()) {
          state = Superstates.IDLE;
        }

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
        } else if (requestIntakeAlgaeFloor
            && !endEffector.hasAlgae()
            && arm.atSetpoint()
            && elevator.atSetpoint()) {
          state = Superstates.INTAKE_ALGAE_FLOOR;
        } else if (requestDescoreAlgae) {
          state = Superstates.DESCORE_ALGAE;
        } else if (requestPreClimb && DriverStation.getMatchTime() < 30.1) {
          state = Superstates.PRECLIMB;
        }

        // TODO
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
        endEffector.algaeHold();
        if (requestEject) {
          state = Superstates.EJECT;
        } else if (!endEffector.hasAlgae()) {
          state = Superstates.IDLE;
        } else {
          arm.algaeHold();
          endEffector.algaeHold();
        }
        if (requestAlgaePrescore) {
          state = Superstates.ALGAE_PRESCORE;
        }

        // TODO
        break;
      case ALGAE_PRESCORE:
        arm.scoreAlgae();
        if (!requestAlgaePrescore) {
          state = Superstates.ALGAE_IDLE;
        } else if (requestAlgaeScore) {
          state = Superstates.ALGAE_SCORE;
        }

        // TODO
        break;
      case ALGAE_SCORE:
        endEffector.releaseAlgae();
        if (!endEffector.hasAlgae() || !requestAlgaePrescore) {
          state = Superstates.SAFE_SCORE_ALGAE_RETRACT;
        } else if (requestSafeScoreAlgaeRetract) {
          state = Superstates.ALGAE_IDLE;
        }

        // TODO
        break;
      case INTAKE_ALGAE_FLOOR: // Needs to move up then arm out then back down
        elevator.algaeGround();
        arm.algaeGround();
        endEffector.intakeAlgae();
        if (endEffector.hasAlgae()) {
          state = Superstates.ALGAE_IDLE;
        } else if (!requestIntakeAlgaeFloor) {
          state = Superstates.IDLE;
        } else if (requestIdle) {
          state = Superstates.IDLE;
        }

        // TODO
        break;
      case DESCORE_ALGAE:
        arm.algaeReef();
        elevator.algaeReef(algaeLevel);
        endEffector.intakeAlgae();
        // TODO
        break;
      case END_EFFECTOR_CORAL_PICKUP:
        if (indexer.isCoralDetectedPickupArea()) {
        elevator.pickupCoral();
        if (elevator.atSetpoint()) {
        endEffector.intakeCoral();
        }}

        if (endEffector.hasCoral() && elevator.atSetpoint()) {
          state = Superstates.CORAL_HELD;
        } else if (!endEffector.hasCoral() && !indexer.isCoralDetectedPickupArea() && elevator.atSetpoint()) {
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
        // TODO
        break;
      case PRESCORE_CORAL:
        arm.prescoreCoral(coralLevel);
        elevator.prescoreCoral(coralLevel);

        if (requestScoreCoral && arm.atSetpoint() && elevator.atSetpoint()
        ) {
          state = Superstates.SCORE_CORAL;
        } else if (!requestPrescoreCoral) {
          state = Superstates.CORAL_HELD;
        } 
        // TODO
        break;
      case SCORE_CORAL: // TODO When have elevator
        arm.scoreCoral(coralLevel);
        elevator.scoreCoral(coralLevel);

        endEffector.releaseCoral();
        if (!requestScoreCoral) {
          state = Superstates.IDLE;
        }
        if (!endEffector.hasCoral()) {
          state = Superstates.IDLE;
        } else if (endEffector.hasCoral()) {
          state = Superstates.CORAL_HELD;
        }
        // TODO
        break;
      case SAFE_SCORE_ALGAE_RETRACT:
        endEffector.idle();
        arm.safeBargeRetract();
        elevator.safeBargeRetract();

        if (arm.atSetpoint() && elevator.atSetpoint() && !endEffector.hasAlgae()) {
          state = Superstates.IDLE;
        }
        if (endEffector.hasAlgae() && arm.atSetpoint() && elevator.atSetpoint()) {
          state = Superstates.ALGAE_IDLE;
        }
        // TODO
        break;
      case PRECLIMB:

        // TODO
        break;
      case CLIMB:
        // TODO
        break;
    }
  }

  public boolean isHomeButtonPressed() {
    return true; // TODO add actual button logic
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestEject = false;
    requestAlgaeIdle = false;
    requestAlgaePrescore = false;
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

  public void requestOperationMode(OperationMode mode){
    unsetAllRequests();
    requestswitchOperationMode = true;
  }

  public boolean isAutoOperationMode(){
    if (mode == OperationMode.Auto){
      return true;
    } else {
      return false;
    }
  }

  public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void requestEject() {
    unsetAllRequests();
    requestEject = true;
  }

  public void requestAlgaeIdle() {
    unsetAllRequests();
    requestAlgaeIdle = true;
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

  public void requestIntakeAlgaeReef(Level algaeLevel) {
    unsetAllRequests();
    requestIntakeAlgaeFloor = true;
  }

  public void requestDescoreAlgae() {
    unsetAllRequests();
    requestDescoreAlgae = true;
  }

  public void requestEndEffectorCoralPickup() {
    unsetAllRequests();
    requestEndEffectorCoralPickup = true;
  }

  public void requestCoralHeld() {
    unsetAllRequests();
    requestCoralHeld = true;
  }

  public void requestPrescoreCoral() {
    unsetAllRequests();
    requestPrescoreCoral = true;
  }

  public void requestScoreCoral() {
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
    return elevator.getHeightMeters();
  }

  public double getArmAngle() {
    return arm.getAngleDegrees();
  }

  public Superstates getState() {
    return state;
  }

  public void getReefStatus(){
    //TODO when get vision working
    //return visionPoseEstimation.getReefStatus()
  }

  public IntakeSuperstructure getIntakeSuperstructure() {
    return intakeSuperstructure;
  }

  //Other Methods are related to Vision Pose Estimation
}
