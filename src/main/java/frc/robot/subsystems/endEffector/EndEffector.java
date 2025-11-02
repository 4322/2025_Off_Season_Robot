package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BabyTunerX;
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
  private boolean requestIntakeAlgae;
  private boolean requestIntakeCoral;
  private boolean requestReleaseAlgae;
  private boolean requestReleaseCoralNormal;
  private boolean requestReleaseCoralL1;
  private boolean requestEject;
  private boolean holdAlgae;
  private boolean holdCoral;
  private boolean dropCoral;

  private boolean coralHeld = false;
  private boolean algaeHeld = false;
  private boolean isPiecePickupDetected = false;

  private Timer intakingTimer = new Timer();
  private Timer releasingTimer = new Timer();

  public enum EndEffectorStates {
    IDLE,
    INTAKE_ALGAE,
    INTAKE_CORAL,
    HOLD_ALGAE,
    HOLD_CORAL,
    RELEASE_ALGAE,
    RELEASE_CORAL_NORMAL,
    RELEASE_CORAL_L1,
    INTAKING_CORAL, // This and below state used to give delay before reducing intake voltage to
    // allow a piece to fully come in
    INTAKING_ALGAE,
    EJECT,
    DROP_REPICKUP,
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
    Logger.recordOutput("End Effector/coralHeld", coralHeld);
    Logger.recordOutput("End Effector/algaeHeld", algaeHeld);

    // Currently unused
    isPiecePickupDetected =
        currentDetectionDebouncer.calculate(inputs.statorCurrentAmps)
            && velocityDetectionDebouncer.calculate(inputs.speedRotationsPerSec);

    Logger.recordOutput("End Effector/isPiecePickupDetected", isPiecePickupDetected());

    if (Constants.endEffectorMode == SubsystemMode.TUNING) {
      BabyTunerX.run(0, io.getTalonFX(), "End-Effector", inputs.speedRotationsPerSec, "rot/sec");
      return;
    }

    switch (state) {
      case IDLE:
        // End effector doesn't do anything
        io.stop();
        if (requestIntakeAlgae) {
          state = EndEffectorStates.INTAKE_ALGAE;
        } else if (requestIntakeCoral) {
          state = EndEffectorStates.INTAKE_CORAL;
        } else if (inputs.isCoralProximityDetected) {
          state = EndEffectorStates.INTAKE_CORAL;
        } else if (requestEject) {
          unsetAllRequests();
          state = EndEffectorStates.EJECT;
        }
        break;
      case INTAKE_ALGAE:
        // Intake algae voltage; Transitions to INTAKING_ALGAE once algae is detected to be pressed
        // against end effector
        if (requestIdle) {
          state = EndEffectorStates.IDLE;
          algaeHeld = false;
        } else {
          io.setVoltage(Constants.EndEffector.algaeIntakeVolts);
          if (inputs.isAlgaeProximityDetected) {
            state = EndEffectorStates.INTAKING_ALGAE;
            intakingTimer.stop();
            intakingTimer.reset();
          }
        }

        break;
      case INTAKE_CORAL:
        // Intake coral voltage; Transitions to INTAKING_CORAL once coral is touching end effector
        io.setVoltage(Constants.EndEffector.coralIntakeVolts);
        if (inputs.isCoralProximityDetected) {
          state = EndEffectorStates.INTAKING_CORAL;
          intakingTimer.stop();
          intakingTimer.reset();
        }
        if (requestIdle) {
          state = EndEffectorStates.IDLE;
          coralHeld = false;
        }
        break;
      case INTAKING_ALGAE:
        // Intake algae voltage; Waits algaeIntakingDelaySeconds, then checks whether algae is held
        // or not
        if (intakingTimer.isRunning()) {
          if (intakingTimer.hasElapsed(Constants.EndEffector.algaeIntakingDelaySeconds)) {
            if (inputs.sensorProximity > Constants.EndEffector.algaeProximityThreshold) {
              algaeHeld = false;
              state = EndEffectorStates.IDLE;
            } else if (inputs.sensorProximity
                < Constants.EndEffector.algaeProximityThresholdIntake) {
              state = EndEffectorStates.HOLD_ALGAE;
              algaeHeld = true;
              intakingTimer.stop();
              intakingTimer.reset();
            }
          }

        } else {
          intakingTimer.start();
        }
        break;
      case INTAKING_CORAL:
        // Intake coral voltage; Waits coralIntakingDelaySeconds then goes to HOLD_CORAL
        if (intakingTimer.isRunning()) {
          if (intakingTimer.hasElapsed(Constants.EndEffector.coralIntakingDelaySeconds)) {
            state = EndEffectorStates.HOLD_CORAL;
            coralHeld = true;
            intakingTimer.stop();
            intakingTimer.reset();
          }
        } else {
          intakingTimer.start();
        }
        break;
      case HOLD_ALGAE:
        if (intakingTimer.isRunning()) {
          intakingTimer.stop();
          intakingTimer.reset();
        }

        if (RobotContainer.getSuperstructure().getArmAngle() <= 90) {
          io.setVoltage(Constants.EndEffector.maxAlgaeHoldVolts);
        } else {
          io.setVoltage(
              Math.abs(
                          Math.sin(
                              Units.degreesToRadians(
                                  RobotContainer.getSuperstructure().getArmAngle())))
                      * (Constants.EndEffector.maxAlgaeHoldVolts
                          - Constants.EndEffector.minAlgaeHoldVolts)
                  + Constants.EndEffector.minAlgaeHoldVolts);
        }
        if (requestReleaseAlgae) {
          state = EndEffectorStates.RELEASE_ALGAE;
        } else if (requestEject) {
          state = EndEffectorStates.EJECT;
        } else if (inputs.sensorProximity > Constants.EndEffector.algaeProximityThreshold) {
          state = EndEffectorStates.IDLE;
          algaeHeld = false;
        }
        break;
      case HOLD_CORAL:
        io.setVoltage(Constants.EndEffector.coralHoldVolts);
        if (requestReleaseCoralNormal) {
          state = EndEffectorStates.RELEASE_CORAL_NORMAL;
        } else if (requestReleaseCoralL1) {
          state = EndEffectorStates.RELEASE_CORAL_L1;
        } else if (requestEject) {
          state = EndEffectorStates.EJECT;
        } else if (inputs.sensorProximity > Constants.EndEffector.coralProximityThreshold) {
          state = EndEffectorStates.IDLE;
          coralHeld = false;
        } else if (dropCoral) {
          state = EndEffectorStates.DROP_REPICKUP;
          coralHeld = false;
        }
        break;
      case RELEASE_ALGAE:
        if (!releasingTimer.isRunning()) {
          releasingTimer.reset();
          releasingTimer.start();
        }
        io.setVoltage(Constants.EndEffector.algaeReleaseVolts);
        if (holdAlgae) {
          state = EndEffectorStates.HOLD_ALGAE;
        } else if (!inputs.isAlgaeProximityDetected
            && releasingTimer.hasElapsed(Constants.EndEffector.algaeReleasingDelaySeconds)) {
          state = EndEffectorStates.IDLE;
          algaeHeld = false;
          releasingTimer.stop();
          releasingTimer.reset();
        } else if (inputs.isAlgaeProximityDetected) {
          state = EndEffectorStates.HOLD_ALGAE;
        }
        break;
      case RELEASE_CORAL_NORMAL:
        if (!releasingTimer.isRunning()) {
          releasingTimer.reset();
          releasingTimer.start();
        }
        io.setVoltage(Constants.EndEffector.coralReleaseVolts);
        if (holdCoral) {
          state = EndEffectorStates.HOLD_CORAL;
          releasingTimer.stop();
          releasingTimer.reset();
        } else if ((!inputs.isCoralProximityDetected
            && releasingTimer.hasElapsed(Constants.EndEffector.coralReleasingDelaySeconds))) {
          state = EndEffectorStates.IDLE;
          coralHeld = false;
          releasingTimer.stop();
          releasingTimer.reset();
        }
        break;
      case RELEASE_CORAL_L1:
        if (!releasingTimer.isRunning()) {
          releasingTimer.reset();
          releasingTimer.start();
        }
        io.setVoltage(Constants.EndEffector.coralReleaseVoltsL1);
        if (holdCoral) {
          state = EndEffectorStates.HOLD_CORAL;
          releasingTimer.stop();
          releasingTimer.reset();
        } else if ((!inputs.isCoralProximityDetected
            && releasingTimer.hasElapsed(Constants.EndEffector.coralReleasingDelaySeconds))) {
          state = EndEffectorStates.IDLE;
          coralHeld = false;
          releasingTimer.stop();
          releasingTimer.reset();
        }
        break;
      case EJECT:
        if (holdAlgae) {
          state = EndEffectorStates.HOLD_ALGAE;
        } else if (holdCoral) {
          state = EndEffectorStates.HOLD_CORAL;
        } else if (ClockUtil.inBound(
            RobotContainer.getSuperstructure().getArmAngle(),
            Constants.Arm.ejectDeg - Constants.Arm.setpointToleranceDegreesEject,
            Constants.Arm.ejectDeg + Constants.Arm.setpointToleranceDegreesEject,
            true)) /*TODO set acual values*/ {
          io.setVoltage(Constants.EndEffector.ejectVolts);
          if ((!inputs.isCoralProximityDetected && !inputs.isAlgaeProximityDetected)) {
            state = EndEffectorStates.IDLE;
            coralHeld = false;
            algaeHeld = false;
          }
        }
        break;
      case DROP_REPICKUP:
        io.setVoltage(Constants.EndEffector.ejectVolts);
        if ((!inputs.isCoralProximityDetected && !inputs.isAlgaeProximityDetected)) {
          dropCoral = false;
          state = EndEffectorStates.IDLE;
        }
        break;
    }
  }

  public void idle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void dropCoral() {
    unsetAllRequests();
    dropCoral = true;
  }

  public void intakeAlgae() {
    unsetAllRequests();
    requestIntakeAlgae = true;
    io.simAlgaeHeld();
  }

  public void intakeCoral() {
    unsetAllRequests();
    requestIntakeCoral = true;
    io.simCoralHeld();
  }

  public void releaseAlgae() {
    unsetAllRequests();
    requestReleaseAlgae = true;
    io.simAlgaeReleased();
  }

  public void releaseCoralNormal() {
    unsetAllRequests();
    requestReleaseCoralNormal = true;
    io.simCoralReleased();
  }

  public void releaseCoralL1() {
    unsetAllRequests();
    requestReleaseCoralL1 = true;
    io.simCoralReleased();
  }

  public void holdAlgae() {
    unsetAllRequests();
    holdAlgae = true;
  }

  public void holdCoral() {
    unsetAllRequests();
    holdCoral = true;
  }

  public void eject() {
    unsetAllRequests();
    requestEject = true;
  }

  public boolean hasAlgae() {
    return algaeHeld;
  }

  public boolean hasCoral() {
    return coralHeld;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestIntakeAlgae = false;
    requestIntakeCoral = false;
    requestReleaseAlgae = false;
    requestReleaseCoralNormal = false;
    requestReleaseCoralL1 = false;
    requestEject = false;
    holdAlgae = false;
    holdCoral = false;
    dropCoral = false;
  }

  public boolean isPiecePickupDetected() {
    return isPiecePickupDetected;
  }

  public EndEffectorStates getState() {
    return state;
  }
}
