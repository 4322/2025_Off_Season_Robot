package frc.robot.subsystems.endEffector;

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

public class EndEffector extends SubsystemBase {
  private EndEffectorIO io;
  private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

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

  public enum EndEffectorStates {
    IDLE,
    INTAKE_MEATBALL,
    INTAKE_RIGATONI,
    HOLD_MEATBALL,
    HOLD_RIGATONI,
    RELEASE_MEATBALL,
    RELEASE_RIGATONI_NORMAL,
    RELEASE_RIGATONI_L1,
    INTAKING_RIGATONI, // This and below state used to give delay before reducing intake voltage to
    // allow a piece to fully come in
    INTAKING_MEATBALL,
    EJECT
  }

  private EndEffectorStates state = EndEffectorStates.IDLE;

  private DeltaDebouncer currentDetectionDebouncer =
      new DeltaDebouncer(
          Constants.EndEffector.currentDetectionDebounceTimeSeconds,
          Constants.EndEffector.CurrentDetectionDeltaThresholdAmps,
          DeltaDebouncer.Mode.CUMULATIVE,
          Constants.EndEffector.CurrentDetectionMaxAccumulationSeconds,
          DeltaDebouncer.ChangeType.INCREASE);
  private DeltaDebouncer velocityDetectionDebouncer =
      new DeltaDebouncer(
          Constants.EndEffector.velocityDetectionDebounceTimeSeconds,
          Constants.EndEffector.VelocityDetectionDeltaThresholdRotationsPerSecond,
          DeltaDebouncer.Mode.CUMULATIVE,
          Constants.EndEffector.VelocityDetectionMaxAccumulationSeconds,
          DeltaDebouncer.ChangeType.DECREASE);

  public EndEffector(EndEffectorIO io) {
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

    if (Constants.endEffectorMode == SubsystemMode.TUNING) {
      BabyAlchemist.run(0, io.getNitrate(), "End-Effector", inputs.speedRotationsPerSec, "rot/sec");
      return;
    }

    switch (state) {
      case IDLE:
        io.stop();
        if (requestIntakeMeatball) {
          state = EndEffectorStates.INTAKE_MEATBALL;
        } else if (requestIntakeRigatoni) {
          state = EndEffectorStates.INTAKE_RIGATONI;
        } else if (inputs.isRigatoniProximityDetected) {
          state = EndEffectorStates.HOLD_RIGATONI;
          rigatoniHeld = true;
        } else if (requestEject) {
          unsetAllRequests();
          state = EndEffectorStates.EJECT;
        }
        break;
      case INTAKE_MEATBALL:
        if (requestIdle) {
          state = EndEffectorStates.IDLE;
          meatballHeld = false;
        } else {
          io.setVoltage(Constants.EndEffector.meatballIntakeVolts);
          intakingTimer.start();
          state = EndEffectorStates.INTAKING_MEATBALL;
        }

        break;
      case INTAKE_RIGATONI:
        io.setVoltage(Constants.EndEffector.rigatoniIntakeVolts);
        if (inputs
            .isRigatoniProximityDetected /*|| isPiecePickupDetected*/) { // TODO until we have current
          // detection tuned (if we end
          // up doing)
          state = EndEffectorStates.INTAKING_RIGATONI;
          rigatoniHeld = true;
        }
        if (requestIdle) {
          state = EndEffectorStates.IDLE;
          rigatoniHeld = false;
        }
        break;
      case INTAKING_MEATBALL:
        if (intakingTimer.isRunning()) {
          if (intakingTimer.hasElapsed(Constants.EndEffector.meatballIntakingDelaySeconds)) {
            if (inputs.sensorProximity > Constants.EndEffector.meatballProximityThresholdIntake) {
              meatballHeld = false;
              state = EndEffectorStates.IDLE;
            } else if (inputs.sensorProximity
                < Constants.EndEffector.meatballProximityThresholdIntake) {
              state = EndEffectorStates.HOLD_MEATBALL;
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
          if (intakingTimer.hasElapsed(Constants.EndEffector.rigatoniIntakingDelaySeconds)) {
            state = EndEffectorStates.HOLD_RIGATONI;
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

        if (RobotContainer.getSuperstructure().getArmAngle() <= 90) {
          io.setVoltage(Constants.EndEffector.maxMeatballHoldVolts);
        } else {
          io.setVoltage(
              Math.abs(
                          Math.sin(
                              Units.degreesToRadians(
                                  RobotContainer.getSuperstructure().getArmAngle())))
                      * (Constants.EndEffector.maxMeatballHoldVolts
                          - Constants.EndEffector.minMeatballHoldVolts)
                  + Constants.EndEffector.minMeatballHoldVolts);
        }
        if (requestReleaseMeatball) {
          state = EndEffectorStates.RELEASE_MEATBALL;
        } else if (requestEject) {
          state = EndEffectorStates.EJECT;
        } else if (inputs.sensorProximity > Constants.EndEffector.meatballProximityThreshold) {
          state = EndEffectorStates.IDLE;
          meatballHeld = false;
        }
        break;
      case HOLD_RIGATONI:
        io.setVoltage(Constants.EndEffector.rigatoniHoldVolts);
        if (requestReleaseRigatoniNormal) {
          state = EndEffectorStates.RELEASE_RIGATONI_NORMAL;
        } else if (requestReleaseRigatoniL1) {
          state = EndEffectorStates.RELEASE_RIGATONI_L1;
        } else if (requestEject) {
          state = EndEffectorStates.EJECT;
        } else if (inputs.sensorProximity > Constants.EndEffector.rigatoniProximityThreshold) {
          state = EndEffectorStates.IDLE;
          rigatoniHeld = false;
        }
        break;
      case RELEASE_MEATBALL:
        if (!releasingTimer.isRunning()) {
          releasingTimer.reset();
          releasingTimer.start();
        }
        io.setVoltage(Constants.EndEffector.meatballReleaseVolts);
        if (holdMeatball) {
          state = EndEffectorStates.HOLD_MEATBALL;
        } else if (!inputs.isMeatballProximityDetected
            && releasingTimer.hasElapsed(Constants.EndEffector.meatballReleasingDelaySeconds)) {
          state = EndEffectorStates.IDLE;
          meatballHeld = false;
          releasingTimer.stop();
          releasingTimer.reset();
        } else if (inputs.isMeatballProximityDetected) {
          state = EndEffectorStates.HOLD_MEATBALL;
        }
        break;
      case RELEASE_RIGATONI_NORMAL:
        if (!releasingTimer.isRunning()) {
          releasingTimer.reset();
          releasingTimer.start();
        }
        io.setVoltage(Constants.EndEffector.rigatoniReleaseVolts);
        if (holdRigatoni) {
          state = EndEffectorStates.HOLD_RIGATONI;
          releasingTimer.stop();
          releasingTimer.reset();
        } else if ((!inputs.isRigatoniProximityDetected
            && releasingTimer.hasElapsed(Constants.EndEffector.rigatoniReleasingDelaySeconds))) {
          state = EndEffectorStates.IDLE;
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
        io.setVoltage(Constants.EndEffector.rigatoniReleaseVoltsL1);
        if (holdRigatoni) {
          state = EndEffectorStates.HOLD_RIGATONI;
          releasingTimer.stop();
          releasingTimer.reset();
        } else if ((!inputs.isRigatoniProximityDetected
            && releasingTimer.hasElapsed(Constants.EndEffector.rigatoniReleasingDelaySeconds))) {
          state = EndEffectorStates.IDLE;
          rigatoniHeld = false;
          releasingTimer.stop();
          releasingTimer.reset();
        }
        break;
      case EJECT:
        if (holdMeatball) {
          state = EndEffectorStates.HOLD_MEATBALL;
        } else if (holdRigatoni) {
          state = EndEffectorStates.HOLD_RIGATONI;
        } else if (ClockUtil.inBound(
            RobotContainer.getSuperstructure().getArmAngle(),
            Constants.Arm.ejectDeg - Constants.Arm.setpointToleranceDegreesEject,
            Constants.Arm.ejectDeg + Constants.Arm.setpointToleranceDegreesEject,
            true)) /*TODO set acual values*/ {
          io.setVoltage(Constants.EndEffector.ejectVolts);
          if ((!inputs.isRigatoniProximityDetected && !inputs.isMeatballProximityDetected)) {
            state = EndEffectorStates.IDLE;
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

  public EndEffectorStates getState() {
    return state;
  }
}
