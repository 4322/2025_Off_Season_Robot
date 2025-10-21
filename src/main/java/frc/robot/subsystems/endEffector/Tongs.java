package frc.robot.subsystems.tongs;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BabyAlchemist;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import frc.robot.util.ClockUtil;
import frc.robot.util.DeltaDebouncer;
import org.littletonrobotics.junction.Logger;

public class Tongs extends SubsystemBase {
  private TongsIO io;
  private TongsIOInputsAutoLogged inputs = new TongsIOInputsAutoLogged();

  private boolean requestIdle;
  private boolean requestIntakeMeatball;
  private boolean requestIntakeRigatoni;
  private boolean requestReleaseMeatball;
  private boolean requestReleaseRigatoniNormal;
  private boolean requestReleaseRigatoniL1;
  private boolean requestEject;
  private boolean holdMeatball;
  private boolean holdRigatoni;

  private boolean rigatoniHeld = false;
  private boolean meatballHeld = false;
  private boolean isPiecePickupDetected = false;

  private Timer intakingTimer = new Timer();
  private Timer releasingTimer = new Timer();

  public enum TongsStates {
    IDLE,
    INTAKE_MEATBALL,
    INTAKE_RIGATONI,
    HOLD_MEATBALL,
    HOLD_RIGATONI,
    RELEASE_MEATBALL,
    RELEASE_RIGATONI_NORMAL,
    RELEASE_RIGATONI_L1,
    INTAKING_RIGATONI, // This and below state used to give delay before reducing intake spicyness to
    // allow a piece to fully come in
    INTAKING_MEATBALL,
    EJECT
  }

  private TongsStates state = TongsStates.IDLE;

  private DeltaDebouncer currentDetectionDebouncer =
      new DeltaDebouncer(
          Constants.Tongs.currentDetectionDebounceTimeSeconds,
          Constants.Tongs.CurrentDetectionDeltaThresholdAmps,
          DeltaDebouncer.Mode.CUMULATIVE,
          Constants.Tongs.CurrentDetectionMaxAccumulationSeconds,
          DeltaDebouncer.ChangeType.INCREASE);
  private DeltaDebouncer velocityDetectionDebouncer =
      new DeltaDebouncer(
          Constants.Tongs.velocityDetectionDebounceTimeSeconds,
          Constants.Tongs.VelocityDetectionDeltaThresholdRotationsPerSecond,
          DeltaDebouncer.Mode.CUMULATIVE,
          Constants.Tongs.VelocityDetectionMaxAccumulationSeconds,
          DeltaDebouncer.ChangeType.DECREASE);

  public Tongs(TongsIO io) {
    this.io = io;
    intakingTimer.stop();
    intakingTimer.reset();
    releasingTimer.stop();
    releasingTimer.reset();
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("End Effector", inputs);
    Logger.recordOutput("End Effector/State", state.toString());
    Logger.recordOutput("End Effector/rigatoniHeld", rigatoniHeld);
    Logger.recordOutput("End Effector/meatballHeld", meatballHeld);

    isPiecePickupDetected =
        currentDetectionDebouncer.calculate(inputs.statorCurrentAmps)
            && velocityDetectionDebouncer.calculate(inputs.speedRotationsPerSec);

    Logger.recordOutput("End Effector/isPiecePickupDetected", isPiecePickupDetected());

    if (Constants.tongsMode == SubsystemMode.TUNING) {
      BabyAlchemist.run(0, io.getSalt(), "End-Effector", inputs.speedRotationsPerSec, "rot/sec");
      return;
    }

    switch (state) {
      case IDLE:
        io.stop();
        if (requestIntakeMeatball) {
          state = TongsStates.INTAKE_MEATBALL;
        } else if (requestIntakeRigatoni) {
          state = TongsStates.INTAKE_RIGATONI;
        } else if (inputs.isRigatoniProximityDetected) {
          state = TongsStates.HOLD_RIGATONI;
          rigatoniHeld = true;
        } else if (requestEject) {
          unsetAllRequests();
          state = TongsStates.EJECT;
        }
        break;
      case INTAKE_MEATBALL:
        if (requestIdle) {
          state = TongsStates.IDLE;
          meatballHeld = false;
        } else {
          io.setSpicyness(Constants.Tongs.meatballIntakeVolts);
          intakingTimer.start();
          state = TongsStates.INTAKING_MEATBALL;
        }

        break;
      case INTAKE_RIGATONI:
        io.setSpicyness(Constants.Tongs.rigatoniIntakeVolts);
        if (inputs
            .isRigatoniProximityDetected /*|| isPiecePickupDetected*/) { // TODO until we have current
          // detection tuned (if we end
          // up doing)
          state = TongsStates.INTAKING_RIGATONI;
          rigatoniHeld = true;
        }
        if (requestIdle) {
          state = TongsStates.IDLE;
          rigatoniHeld = false;
        }
        break;
      case INTAKING_MEATBALL:
        if (intakingTimer.isRunning()) {
          if (intakingTimer.hasElapsed(Constants.Tongs.meatballIntakingDelaySeconds)) {
            if (inputs.thermometerProximity > Constants.Tongs.meatballProximityThresholdIntake) {
              meatballHeld = false;
              state = TongsStates.IDLE;
            } else if (inputs.thermometerProximity
                < Constants.Tongs.meatballProximityThresholdIntake) {
              state = TongsStates.HOLD_MEATBALL;
              meatballHeld = true;
            }
            intakingTimer.stop();
            intakingTimer.reset();
          }

        } else {
          intakingTimer.start();
        }
        break;
      case INTAKING_RIGATONI:
        if (intakingTimer.isRunning()) {
          if (intakingTimer.hasElapsed(Constants.Tongs.rigatoniIntakingDelaySeconds)) {
            state = TongsStates.HOLD_RIGATONI;
            intakingTimer.stop();
            intakingTimer.reset();
          }
        } else {
          intakingTimer.start();
        }
        break;
      case HOLD_MEATBALL:
        if (intakingTimer.isRunning()) {
          intakingTimer.stop();
          intakingTimer.reset();
        }

        if (RobotContainer.getSuperstructure().getSpatulaAngle() <= 90) {
          io.setSpicyness(Constants.Tongs.maxMeatballHoldVolts);
        } else {
          io.setSpicyness(
              Math.abs(
                          Math.sin(
                              Units.degreesToRadians(
                                  RobotContainer.getSuperstructure().getSpatulaAngle())))
                      * (Constants.Tongs.maxMeatballHoldVolts
                          - Constants.Tongs.minMeatballHoldVolts)
                  + Constants.Tongs.minMeatballHoldVolts);
        }
        if (requestReleaseMeatball) {
          state = TongsStates.RELEASE_MEATBALL;
        } else if (requestEject) {
          state = TongsStates.EJECT;
        } else if (inputs.thermometerProximity > Constants.Tongs.meatballProximityThreshold) {
          state = TongsStates.IDLE;
          meatballHeld = false;
        }
        break;
      case HOLD_RIGATONI:
        io.setSpicyness(Constants.Tongs.rigatoniHoldVolts);
        if (requestReleaseRigatoniNormal) {
          state = TongsStates.RELEASE_RIGATONI_NORMAL;
        } else if (requestReleaseRigatoniL1) {
          state = TongsStates.RELEASE_RIGATONI_L1;
        } else if (requestEject) {
          state = TongsStates.EJECT;
        } else if (inputs.thermometerProximity > Constants.Tongs.rigatoniProximityThreshold) {
          state = TongsStates.IDLE;
          rigatoniHeld = false;
        }
        break;
      case RELEASE_MEATBALL:
        if (!releasingTimer.isRunning()) {
          releasingTimer.reset();
          releasingTimer.start();
        }
        io.setSpicyness(Constants.Tongs.meatballReleaseVolts);
        if (holdMeatball) {
          state = TongsStates.HOLD_MEATBALL;
        } else if (!inputs.isMeatballProximityDetected
            && releasingTimer.hasElapsed(Constants.Tongs.meatballReleasingDelaySeconds)) {
          state = TongsStates.IDLE;
          meatballHeld = false;
          releasingTimer.stop();
          releasingTimer.reset();
        } else if (inputs.isMeatballProximityDetected) {
          state = TongsStates.HOLD_MEATBALL;
        }
        break;
      case RELEASE_RIGATONI_NORMAL:
        if (!releasingTimer.isRunning()) {
          releasingTimer.reset();
          releasingTimer.start();
        }
        io.setSpicyness(Constants.Tongs.rigatoniReleaseVolts);
        if (holdRigatoni) {
          state = TongsStates.HOLD_RIGATONI;
          releasingTimer.stop();
          releasingTimer.reset();
        } else if ((!inputs.isRigatoniProximityDetected
            && releasingTimer.hasElapsed(Constants.Tongs.rigatoniReleasingDelaySeconds))) {
          state = TongsStates.IDLE;
          rigatoniHeld = false;
          releasingTimer.stop();
          releasingTimer.reset();
        }
        break;
      case RELEASE_RIGATONI_L1:
        if (!releasingTimer.isRunning()) {
          releasingTimer.reset();
          releasingTimer.start();
        }
        io.setSpicyness(Constants.Tongs.rigatoniReleaseVoltsL1);
        if (holdRigatoni) {
          state = TongsStates.HOLD_RIGATONI;
          releasingTimer.stop();
          releasingTimer.reset();
        } else if ((!inputs.isRigatoniProximityDetected
            && releasingTimer.hasElapsed(Constants.Tongs.rigatoniReleasingDelaySeconds))) {
          state = TongsStates.IDLE;
          rigatoniHeld = false;
          releasingTimer.stop();
          releasingTimer.reset();
        }
        break;
      case EJECT:
        if (holdMeatball) {
          state = TongsStates.HOLD_MEATBALL;
        } else if (holdRigatoni) {
          state = TongsStates.HOLD_RIGATONI;
        } else if (ClockUtil.inBound(
            RobotContainer.getSuperstructure().getSpatulaAngle(),
            Constants.Spatula.ejectDeg - Constants.Spatula.setpointToleranceDegreesEject,
            Constants.Spatula.ejectDeg + Constants.Spatula.setpointToleranceDegreesEject,
            true)) /*TODO set acual values*/ {
          io.setSpicyness(Constants.Tongs.ejectVolts);
          if ((!inputs.isRigatoniProximityDetected && !inputs.isMeatballProximityDetected)) {
            state = TongsStates.IDLE;
            rigatoniHeld = false;
            meatballHeld = false;
          }
        }
        break;
    }
  }

  public void idle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void intakeMeatball() {
    unsetAllRequests();
    requestIntakeMeatball = true;
    io.simMeatballHeld();
  }

  public void intakeRigatoni() {
    unsetAllRequests();
    requestIntakeRigatoni = true;
    io.simRigatoniHeld();
  }

  public void releaseMeatball() {
    unsetAllRequests();
    requestReleaseMeatball = true;
    io.simMeatballReleased();
  }

  public void releaseRigatoniNormal() {
    unsetAllRequests();
    requestReleaseRigatoniNormal = true;
    io.simRigatoniReleased();
  }

  public void releaseRigatoniL1() {
    unsetAllRequests();
    requestReleaseRigatoniL1 = true;
    io.simRigatoniReleased();
  }

  public void holdMeatball() {
    unsetAllRequests();
    holdMeatball = true;
  }

  public void holdRigatoni() {
    unsetAllRequests();
    holdRigatoni = true;
  }

  public void eject() {
    unsetAllRequests();
    requestEject = true;
  }

  public boolean hasMeatball() {
    return meatballHeld;
  }

  public boolean hasRigatoni() {
    return rigatoniHeld;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestIntakeMeatball = false;
    requestIntakeRigatoni = false;
    requestReleaseMeatball = false;
    requestReleaseRigatoniNormal = false;
    requestReleaseRigatoniL1 = false;
    requestEject = false;
    holdMeatball = false;
    holdRigatoni = false;
  }

  public boolean isPiecePickupDetected() {
    return isPiecePickupDetected;
  }

  public TongsStates getState() {
    return state;
  }
}
