package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.Logger;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BabyAlchemist;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.ClockUtil;
import frc.robot.util.DeltaDebouncer;

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
  private Arm arm;

  private boolean coralHeld = false;
  private boolean algaeHeld = false;
  private boolean isPiecePickupDetected = false;

  private Timer intakingTimer = new Timer();
  private Timer releasingTimer = new Timer();

  private enum EndEffectorStates {
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
    Logger.recordOutput("End Effector/coralHeld", coralHeld);
    Logger.recordOutput("End Effector/algaeHeld", algaeHeld);

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
        io.stop(IdleMode.kCoast);
        if (requestIntakeAlgae) {
          state = EndEffectorStates.INTAKE_ALGAE;
        } else if (requestIntakeCoral) {
          state = EndEffectorStates.INTAKE_CORAL;
        }
        break;
      case INTAKE_ALGAE:
      if (inputs.sensorProximity > 0.15){
        io.setVoltage(Constants.EndEffector.algaeIntakeVolts);
        intakingTimer.start();
        state = EndEffectorStates.INTAKING_ALGAE;
      }
        if (inputs.isAlgaeProximityDetected || isPiecePickupDetected) {
          state = EndEffectorStates.INTAKING_ALGAE;
          algaeHeld = true;
        }
        if (requestIdle) {
          state = EndEffectorStates.IDLE;
          algaeHeld = false;
        }

        break;
      case INTAKE_CORAL:
        io.setVoltage(Constants.EndEffector.coralIntakeVolts);
        if (inputs.isCoralProximityDetected || isPiecePickupDetected) {
          state = EndEffectorStates.INTAKING_CORAL;
          coralHeld = true;
        }
        if (requestIdle) {
          state = EndEffectorStates.IDLE;
          coralHeld = false;
        }
        break;
      case INTAKING_ALGAE:
        if (intakingTimer.isRunning()) {
          if (intakingTimer.hasElapsed(Constants.EndEffector.algaeIntakingDelaySeconds)) {
            if (inputs.sensorProximity > 0.29){
            algaeHeld = false;
            state = EndEffectorStates.IDLE;
            }
            else if(inputs.sensorProximity > 0.125){
            state = EndEffectorStates.HOLD_ALGAE;
            algaeHeld = true;
            }
            intakingTimer.stop();
            intakingTimer.reset();
          }
          
        } else {
          intakingTimer.start();
        }
      case INTAKING_CORAL:
        if (intakingTimer.isRunning()) {
          if (intakingTimer.hasElapsed(Constants.EndEffector.coralIntakingDelaySeconds)) {
            state = EndEffectorStates.HOLD_CORAL;
            intakingTimer.stop();
            intakingTimer.reset();
          }
        } else {
          intakingTimer.start();
        }
        break;
      case HOLD_ALGAE:
      if (intakingTimer.isRunning()){
      intakingTimer.stop();
      intakingTimer.reset();}

    if (arm.getAngleDegrees() > 180){
      io.setVoltage(1);
    }
    else if (arm.getAngleDegrees() > 135){
      io.setVoltage(1.5);
    }
    else if (arm.getAngleDegrees() > 90){
      io.setVoltage(2);
    }
        if (requestReleaseAlgae) {
          state = EndEffectorStates.RELEASE_ALGAE;
        } else if (requestEject) {
          state = EndEffectorStates.EJECT;
        }

        if (!algaeHeld) {
          state = EndEffectorStates.IDLE;
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
        }

        if (!coralHeld) {
          state = EndEffectorStates.IDLE;
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
            Constants.Arm.ejectDeg - Constants.Arm.setpointToleranceDegrees,
            Constants.Arm.ejectDeg + Constants.Arm.setpointToleranceDegrees,
            true)) /*TODO set acual values*/ {
          io.setVoltage(Constants.EndEffector.ejectVolts);
          if ((!inputs.isCoralProximityDetected && !inputs.isAlgaeProximityDetected)) {
            state = EndEffectorStates.IDLE;
            coralHeld = false;
            algaeHeld = false;
          }
        }
        break;
    }
  }

  public void idle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void intakeAlgae() {
    unsetAllRequests();
    requestIntakeAlgae = true;
  }

  public void intakeCoral() {
    unsetAllRequests();
    requestIntakeCoral = true;
  }

  public void releaseAlgae() {
    unsetAllRequests();
    requestReleaseAlgae = true;
  }

  public void releaseCoralNormal() {
    unsetAllRequests();
    requestReleaseCoralNormal = true;
  }

  public void releaseCoralL1() {
    unsetAllRequests();
    requestReleaseCoralL1 = true;
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
    requestEject = true;
  }

  public boolean hasAlgae() {
    return algaeHeld;
  }

  public boolean hasCoral() {
    return coralHeld;
  }

  public void setNeutralMode(IdleMode mode) {
    io.stop(mode);
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
  }

  public boolean isPiecePickupDetected() {
    return isPiecePickupDetected;
  }
}
