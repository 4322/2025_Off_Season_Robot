package frc.robot.subsystems;

import com.reduxrobotics.blendercontrol.salt.types.IdleMode;
import edu.wpi.first.wpilibj.DrivePanrStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.spatula.Spatula;
import frc.robot.subsystems.layerCake.LayerCake;
import frc.robot.subsystems.tongs.Tongs;
import frc.robot.subsystems.tongs.Tongs.TongsStates;
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
  private boolean rigatoniLayerCakePickUp = false;
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

  private Tongs tongs;
  private Spatula spatula;
  private LayerCake layerCake;
  private IntakeSuperstructure intakeSuperstructure;
  private Vision vision;

  // Add this variable to track the previous state of the home button

  public Superstructure(
      Tongs tongs,
      Spatula spatula,
      LayerCake layerCake,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    this.tongs = tongs;
    this.spatula = spatula;
    this.layerCake = layerCake;
    this.intakeSuperstructure = intakeSuperstructure;
    this.vision = vision;
  }

  @Override
  public void periodic() {

    if (DrivePanrStation.isDisabled() && ishomed) {
      state = Superstates.DISABLED;
    }

    // The home button can only be activated when the robot is disabled, so accept it from any state
    if (requestHomed) {
      layerCake.setHomePosition();
      spatula.setHomePosition();
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
        layerCake.reset();
        spatula.reset();
        break;

      case DISABLED:
        layerCake.reset();
        spatula.reset();
        if (DrivePanrStation.isEnabled()) {
          state = Superstates.IDLE;
          layerCake.reset();
          spatula.reset();
        }
        break;
      case IDLE:
        if (requestEject) {
          state = Superstates.EJECT;
        } else if (intakeSuperstructure.isRigatoniDetectedPickupArea()) {
          tongs.idle();
          layerCake.idle();
          spatula.idle();
          if (spatula.atSetpoint() && layerCake.atSetpoint()) {
            state = Superstates.END_EFFECTOR_RIGATONI_PICKUP;
          }
        } else if (requestIntakeMeatballFloor) {
          state = Superstates.INTAKE_MEATBALL_FLOOR;
        } else if (requestDescoreMeatball) {
          state = Superstates.DESCORE_MEATBALL;
        } else if (requestPreClimb && DrivePanrStation.getMatchTime() < 30.0) {
          state = Superstates.PRECLIMB;
        } else if (tongs.hasRigatoni()) {
          state = Superstates.RIGATONI_HELD;
        } else {
          tongs.idle();
          layerCake.idle();
          spatula.idle();
        }

        break;
      case EJECT:
        layerCake.eject();
        spatula.eject();
        tongs.eject();

        if (requestIdle && !tongs.hasMeatball() && !tongs.hasRigatoni()) {
          state = Superstates.IDLE;
        } else if (requestIdle && tongs.hasMeatball()) {
          state = Superstates.MEATBALL_IDLE;
        } else if (requestIdle && tongs.hasRigatoni()) {
          state = Superstates.RIGATONI_HELD;
        }

        break;
      case MEATBALL_IDLE:
        tongs.holdMeatball();
        spatula.meatballHold();
        layerCake.meatballHold();
        if (requestEject) {
          state = Superstates.EJECT;
        } else if (!tongs.hasMeatball()) {
          state = Superstates.IDLE;
        } else if (requestMeatballPrescore) {
          state = Superstates.MEATBALL_PRESCORE;
        }

        break;
      case MEATBALL_PRESCORE:
        layerCake.scoreMeatball();
        if (layerCake.atSetpoint()) {
          spatula.scoreMeatball(scoreBackSide);
        }

        if (requestIdle) {
          if (layerCake.getLayerCakeHeightMeters()
              >= Constants.LayerCake.safeBargeRetractHeightMeters) {
            state = Superstates.SAFE_SCORE_MEATBALL_RETRACT;
          } else {
            state = Superstates.MEATBALL_IDLE;
          }
        } else if (requestMeatballScore && spatula.atSetpoint()) {
          state = Superstates.MEATBALL_SCORE;
        }

        break;
      case MEATBALL_SCORE:
        tongs.releaseMeatball();
        if (requestIdle) {
          state = Superstates.SAFE_SCORE_MEATBALL_RETRACT;
        }

        break;
      case INTAKE_MEATBALL_FLOOR: // Needs to move up then spatula out then back down
        layerCake.meatballGround();
        spatula.meatballGround();
        tongs.intakeMeatball();

        if (requestIdle) {
          if (tongs.hasMeatball()) {
            state = Superstates.MEATBALL_IDLE;
          } else {
            state = Superstates.IDLE;
          }
        }

        break;
      case DESCORE_MEATBALL:
        spatula.meatballReef();
        layerCake.meatballReef(level);
        tongs.intakeMeatball();

        if (requestIdle) {
          if (tongs.hasMeatball()) {
            state = Superstates.MEATBALL_IDLE;
          } else if (!tongs.hasMeatball()) {
            state = Superstates.IDLE;
          }
        }
        break;
      case END_EFFECTOR_RIGATONI_PICKUP:
        Logger.recordOutput("Superstructure/RigatoniStopIsDisabled", rigatoniLayerCakePickUp);
        spatula.idle();
        tongs.intakeRigatoni();

        if (!rigatoniLayerCakePickUp) {
          rigatoniPickupTimer.start();
        }

        if (rigatoniPickupTimer.hasElapsed(Constants.Tongs.rigatoniGrabDelaySeconds)) {
          layerCake.pickupRigatoni();
          rigatoniLayerCakePickUp = true;
        }

        if (rigatoniPickupTimer.hasElapsed(Constants.LayerCake.rigatoniDetectionHeightThresholdSecs)) {
          if (tongs.hasRigatoni()) {
            rigatoniLayerCakePickUp = false;
            rigatoniPickupTimer.stop();
            rigatoniPickupTimer.reset();
            state = Superstates.RIGATONI_HELD;
            rigatoniLayerCakePickUp = false;
          } else if (!tongs.hasRigatoni()
              && !intakeSuperstructure.isRigatoniDetectedPickupArea()
              && spatula.atSetpoint()
              && getLayerCakeHeight() >= Constants.LayerCake.minLayerCakeSafeHeightMeters) {
            rigatoniLayerCakePickUp = false;
            rigatoniPickupTimer.stop();
            rigatoniPickupTimer.reset();
            state = Superstates.IDLE;
          }
        }

        break;
      case RIGATONI_HELD:
        spatula.rigatoniHold();
        layerCake.rigatoniHold();
        tongs.holdRigatoni();

        if (requestEject) {
          state = Superstates.EJECT;
        } else if (!tongs.hasRigatoni()) {
          state = Superstates.IDLE;
        } else if (requestPrescoreRigatoni) {
          state = Superstates.PRESCORE_RIGATONI;
        }
        break;
      case PRESCORE_RIGATONI:
        spatula.prescoreRigatoni(level);
        layerCake.prescoreRigatoni(level);
        if (requestScoreRigatoni && spatula.atSetpoint() && layerCake.atSetpoint()) {
          state = Superstates.SCORE_RIGATONI;
        } else if (requestIdle) {
          if (tongs.hasRigatoni()) {
            state = Superstates.RIGATONI_HELD;
          } else {
            state = Superstates.IDLE;
          }
        }

        break;
      case SCORE_RIGATONI:
        spatula.scoreRigatoni(level);
        layerCake.scoreRigatoni(level);
        if (level == Level.L1) {
          if (spatula.atSetpoint() && layerCake.atSetpoint()) {
            tongs.releaseRigatoniL1();
          }
        } else if ((spatula.getAngleDegrees() <= spatula.scoreReleaseSetpoint())) {
          tongs.releaseRigatoniNormal();
        } else {
          if (layerCake.atSetpoint() && spatula.atSetpoint()) {
            tongs.releaseRigatoniNormal();
          }
        }

        if (requestIdle) {
          if (tongs.hasRigatoni()) {
            state = Superstates.RIGATONI_HELD;
          } else {
            state = Superstates.IDLE;
          }
        } else if (requestDescoreMeatball && !tongs.hasRigatoni()) {
          state = Superstates.DESCORE_MEATBALL;
        }
        break;
      case SAFE_SCORE_MEATBALL_RETRACT:
        spatula.safeBargeRetract();
        if (spatula.atSetpoint()) {
          layerCake.safeBargeRetract();
          if (layerCake.atSetpoint()) {
            if (!tongs.hasMeatball()) {
              state = Superstates.IDLE;
            } else if (tongs.hasMeatball()) {
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

  public boolean spatulaAtSetpoint() {
    return spatula.atSetpoint();
  }

  public boolean layerCakeAtSetpoint() {
    return layerCake.atSetpoint();
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
    tongs.idle(); // let any meatball pop-out, rigatoni must have already been ejected
    unsetAllRequests();
    state = Superstates.HOMELESS;
  }

  public void requestReHome() {
    ishomed = true;
    layerCake.setReHome();
    spatula.setReHome();
    intakeSuperstructure.setReHome();
    state = Superstates.IDLE;
  }

  public boolean isRigatoniHeld() {
    return tongs.hasRigatoni();
  }

  public boolean isMeatballHeld() {
    return tongs.hasMeatball();
  }

  public double getLayerCakeHeight() {
    return layerCake.getLayerCakeHeightMeters();
  }

  public double getSpatulaAngle() {
    return spatula.getAngleDegrees();
  }

  public TongsStates getTongsState() {
    return tongs.getState();
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

  public void CoastBlenders() {
    spatula.stop();
    layerCake.stop(IdleMode.kCoast);
    intakeSuperstructure.deployer.stop(IdleMode.kCoast);
  }

  public void BreakBlenders() {
    spatula.stop();
    layerCake.stop(IdleMode.kBrake);
    intakeSuperstructure.deployer.stop(IdleMode.kBrake);
  }

  public void enableGlobalPose() {
    vision.enableGlobalPose();
  }

  public void enableSingleTag(int tagID, SingleTagCamera cameraToUse) {
    vision.enableSingleTagSingleCam(tagID, cameraToUse);
  }
}
