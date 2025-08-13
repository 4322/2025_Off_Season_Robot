package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

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
          if (startTimer.hasElapsed(1)){
          arm.setHome();
          elevator.setHome();
          state = Superstates.IDLE;}
        }
        if (DriverStation.isEnabled()) {
          state = Superstates.IDLE;
        }

        break;
      case IDLE:
        if (arm.atSetpoint() && elevator.atSetpoint()) {
          endEffector.idle();
          arm.idle();
          elevator.idle();
        }

        if (requestEject) {
          state = Superstates.EJECT;
        } else if (indexer.isCoralDetectedPickupArea()) {
          state = Superstates.END_EFFECTOR_CORAL_PICKUP;
        } else if (requestIntakeAlgaeFloor && !endEffector.hasAlgae()) {
          state = Superstates.INTAKE_ALGAE_FLOOR;
        } else if (requestDescoreAlgae) {
          state = Superstates.DESCORE_ALGAE;
        } else if (requestPreClimb) {
          // TODO: Add logic for if there is only 30 seconds left in the match
          state = Superstates.PRECLIMB;
        }

        // TODO
        break;
      case EJECT:
        if (arm.atSetpoint() && elevator.atSetpoint()) {
          endEffector.eject();
          elevator.eject();
          arm.eject();
        }

        if (!requestEject && !endEffector.hasAlgae() && !endEffector.hasCoral()) {
          state = Superstates.IDLE;
        } else if (!requestEject && endEffector.hasAlgae()) {
          state = Superstates.ALGAE_IDLE;
        } else if (!requestEject && endEffector.hasCoral()) {
          state = Superstates.CORAL_HELD;
        } else if (requestIdle) {
          state = Superstates.IDLE;
        }
        // TODO
        break;
      case ALGAE_IDLE:
        if (requestEject) {
          state = Superstates.EJECT;
        }

        if (!endEffector.hasAlgae()) {
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
        } else if (endEffector.hasAlgae()) {
          state = Superstates.ALGAE_IDLE;
        }

        // TODO
        break;
      case INTAKE_ALGAE_FLOOR:
        endEffector.intakeAlgae();
        arm.algaeGround();
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
        arm.algaeReef(algaeLevel);
        elevator.algaeReef(algaeLevel);
        endEffector.intakeAlgae();
        // TODO
        break;
      case END_EFFECTOR_CORAL_PICKUP:
        endEffector.intakeCoral();

        if (endEffector.hasCoral()) {
          state = Superstates.CORAL_HELD;
        } else if (!endEffector.hasCoral()) {
          state = Superstates.IDLE;
        }

        break;
      case CORAL_HELD:
        if (requestEject) {
          state = Superstates.EJECT;
        } else if (!endEffector.hasCoral()) {
          state = Superstates.IDLE;
        } else if (requestPrescoreCoral) {
          state = Superstates.PRESCORE_CORAL;
        }
        if (endEffector.hasCoral()) {
          arm.idle();
        }
        // TODO
        break;
      case PRESCORE_CORAL:
        arm.prescoreCoral(coralLevel);
        elevator.prescoreCoral(coralLevel);

        if (arm.atSetpoint()) {
          state = Superstates.SCORE_CORAL;
        } else if (!requestPrescoreCoral) {
          state = Superstates.CORAL_HELD;
        } else if (requestScoreCoral) {
          state = Superstates.SCORE_CORAL;
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
        if (endEffector.hasAlgae()) {
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

  public double getElevatorHeight() {
    return elevator.getHeightMeters();
  }

  public double getArmAngle() {
    return arm.getAngleDegrees();
  }

  public Superstates getState() {
    return state;
  }
}
