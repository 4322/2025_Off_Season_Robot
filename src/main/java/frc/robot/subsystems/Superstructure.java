package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

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
  private boolean ishomed = false;
  private Timer coralPickupTimer = new Timer();
  private boolean coralElevatorPickUp = false;
  private boolean scoreBackSide = false;

  public enum Superstates {
    HOMELESS,
    DISABLED,
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
        } else if (intakeSuperstructure.isCoralDetectedPickupArea()) {
          endEffector.idle();
          elevator.idle();
          arm.idle();
          if (arm.atSetpoint() && elevator.atSetpoint()) {
            state = Superstates.END_EFFECTOR_CORAL_PICKUP;
          }
        } else if (requestIntakeAlgaeFloor) {
          state = Superstates.INTAKE_ALGAE_FLOOR;
        } else if (requestDescoreAlgae) {
          state = Superstates.DESCORE_ALGAE;
        } else if (requestPreClimb && DriverStation.getMatchTime() < 30.0) {
          state = Superstates.PRECLIMB;
        } else if (endEffector.hasCoral()) {
          state = Superstates.CORAL_HELD;
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

        if (requestIdle && !endEffector.hasAlgae() && !endEffector.hasCoral()) {
          state = Superstates.IDLE;
        } else if (requestIdle && endEffector.hasAlgae()) {
          state = Superstates.ALGAE_IDLE;
        } else if (requestIdle && endEffector.hasCoral()) {
          state = Superstates.CORAL_HELD;
        }

        break;
      case ALGAE_IDLE:
        endEffector.holdAlgae();
        arm.algaeHold();
        elevator.algaeHold();
        if (requestEject) {
          state = Superstates.EJECT;
        } else if (!endEffector.hasAlgae()) {
          state = Superstates.IDLE;
        } else if (requestAlgaePrescore) {
          state = Superstates.ALGAE_PRESCORE;
        }

        break;
      case ALGAE_PRESCORE:
        elevator.scoreAlgae();
        if (elevator.atSetpoint()) {
          arm.scoreAlgae(scoreBackSide);
        }

        if (requestIdle) {
          if (elevator.getElevatorHeightMeters()
              >= Constants.Elevator.safeBargeRetractHeightMeters) {
            state = Superstates.SAFE_SCORE_ALGAE_RETRACT;
          } else {
            state = Superstates.ALGAE_IDLE;
          }
        } else if (requestAlgaeScore && arm.atSetpoint()) {
          state = Superstates.ALGAE_SCORE;
        }

        break;
      case ALGAE_SCORE:
        endEffector.releaseAlgae();
        if (requestIdle) {
          state = Superstates.SAFE_SCORE_ALGAE_RETRACT;
        }

        break;
      case INTAKE_ALGAE_FLOOR: // Needs to move up then arm out then back down
        elevator.algaeGround();
        arm.algaeGround();
        endEffector.intakeAlgae();

        if (requestIdle) {
          if (endEffector.hasAlgae()) {
            state = Superstates.ALGAE_IDLE;
          } else {
            state = Superstates.IDLE;
          }
        }

        break;
      case DESCORE_ALGAE:
        arm.algaeReef();
        elevator.algaeReef(level);
        endEffector.intakeAlgae();

        if (requestIdle) {
          if (endEffector.hasAlgae()) {
            state = Superstates.ALGAE_IDLE;
          } else if (!endEffector.hasAlgae()) {
            state = Superstates.IDLE;
          }
        }
        break;
      case END_EFFECTOR_CORAL_PICKUP:
        Logger.recordOutput("Superstructure/CoralStopIsDisabled", coralElevatorPickUp);
        arm.idle();
        endEffector.intakeCoral();

        if (!coralElevatorPickUp) {
          coralPickupTimer.start();
        }

        if (coralPickupTimer.hasElapsed(Constants.EndEffector.coralGrabDelaySeconds)) {
          elevator.pickupCoral();
          coralElevatorPickUp = true;
        }

        if (coralPickupTimer.hasElapsed(Constants.Elevator.coralDetectionHeightThresholdSecs)) {
          if (endEffector.hasCoral()) {
            coralElevatorPickUp = false;
            coralPickupTimer.stop();
            coralPickupTimer.reset();
            state = Superstates.CORAL_HELD;
            coralElevatorPickUp = false;
          } else if (!endEffector.hasCoral()
              && !intakeSuperstructure.isCoralDetectedPickupArea()
              && arm.atSetpoint()
              && getElevatorHeight() >= Constants.Elevator.minElevatorSafeHeightMeters) {
            coralElevatorPickUp = false;
            coralPickupTimer.stop();
            coralPickupTimer.reset();
            state = Superstates.IDLE;
          }
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
        } else if (requestIdle) {
          if (endEffector.hasCoral()) {
            state = Superstates.CORAL_HELD;
          } else {
            state = Superstates.IDLE;
          }
        }

        break;
      case SCORE_CORAL:
        arm.scoreCoral(level);
        elevator.scoreCoral(level);
        if (level == Level.L1) {
          if (arm.atSetpoint() && elevator.atSetpoint()) {
            endEffector.releaseCoralL1();
          }
        } else if ((arm.getAngleDegrees() <= arm.scoreReleaseSetpoint())) {
          endEffector.releaseCoralNormal();
        } else {
          if (elevator.atSetpoint() && arm.atSetpoint()) {
            endEffector.releaseCoralNormal();
          }
        }

        if (requestIdle) {
          if (endEffector.hasCoral()) {
            state = Superstates.CORAL_HELD;
          } else {
            state = Superstates.IDLE;
          }
        } else if (requestDescoreAlgae && !endEffector.hasCoral()) {
          state = Superstates.DESCORE_ALGAE;
        }
        break;
      case SAFE_SCORE_ALGAE_RETRACT:
        arm.safeBargeRetract();
        if (arm.atSetpoint()) {
          elevator.safeBargeRetract();
          if (elevator.atSetpoint()) {
            if (!endEffector.hasAlgae()) {
              state = Superstates.IDLE;
            } else if (endEffector.hasAlgae()) {
              state = Superstates.ALGAE_IDLE;
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
    requestAlgaePrescore = false;
    requestPrescoreCoral = false;
    requestScoreCoral = false;
    requestPreClimb = false;
    requestClimb = false;
    requestIntakeAlgaeFloor = false;
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

  public void requestAlgaePrescore(boolean prescoreBackSide) {
    unsetAllRequests();
    requestAlgaePrescore = true;
    this.scoreBackSide = prescoreBackSide;
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

  public void requestUnhome() {
    endEffector.idle(); // let any algae pop-out, coral must have already been ejected
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
    arm.enableBrakeMode(false);
    elevator.enableBrakeMode(false);
    intakeSuperstructure.deployer.stop(IdleMode.kCoast);
  }

  public void BreakMotors() {
    arm.enableBrakeMode(true);
    elevator.enableBrakeMode(true);
    intakeSuperstructure.deployer.stop(IdleMode.kBrake);
  }

  public void enableGlobalPose() {
    vision.enableGlobalPose();
  }

  public void enableSingleTag(int tagID, SingleTagCamera cameraToUse) {
    vision.enableSingleTagSingleCam(tagID, cameraToUse);
  }
}
