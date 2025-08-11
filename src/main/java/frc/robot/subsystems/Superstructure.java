package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.vision.Vision;

public class Superstructure extends SubsystemBase {

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

  public enum SuperstructureState {
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

  SuperstructureState currentState = SuperstructureState.START;

  private EndEffector endEffector;
  private Arm arm;
  // TODO wait for Isaac to merge this into main: private Elevator elevator;
  // TODO wait for Ellie to merge this into main: private Climber climber;
  private Drive drive;
  private Vision vision;
  private IntakeSuperstructure intakeSuperstructure;

  public Superstructure(
      EndEffector endEffector,
      Arm arm,
      // TODO wait for Isaac to merge this into main: Elevator elevator,
      // TODO wait for Ellie to merge this into main: Climber climber,
      Drive drive,
      Vision vision,
      IntakeSuperstructure intakeSuperstructure) {
    this.endEffector = endEffector;
    this.arm = arm;
    // TODO wait for Isaac to merge this into main: this.elevator = elevator;
    // TODO wait for Ellie to merge this into main: this.climber = climber;
    this.drive = drive;
    this.vision = vision;
    this.intakeSuperstructure = intakeSuperstructure;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/currentState", currentState.toString());

    switch (currentState) {
      case START:
        break;
      case IDLE:
        endEffector.idle();
        // TODO
        break;
      case EJECT:
        endEffector.eject();

        // TODO
        break;
      case ALGAE_IDLE:

        // TODO
        break;
      case ALGAE_PRESCORE:

        // TODO
        break;
      case ALGAE_SCORE:
        endEffector.releaseAlgae();

        // TODO
        break;
      case INTAKE_ALGAE_FLOOR:
        endEffector.intakeAlgae();

        // TODO
        break;
      case DESCORE_ALGAE:
        endEffector.intakeAlgae();
        // TODO
        break;
      case END_EFFECTOR_CORAL_PICKUP:
        endEffector.intakeCoral();
        // TODO
        break;
      case CORAL_HELD:

        // TODO
        break;
      case PRESCORE_CORAL:

        // TODO
        break;
      case SCORE_CORAL:
        //endEffector.releaseCoral();
        // TODO
        break;
      case SAFE_SCORE_ALGAE_RETRACT:

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




}
