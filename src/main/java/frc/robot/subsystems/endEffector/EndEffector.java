package frc.robot.subsystems.endEffector;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private EndEffectorIO io;
  private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  private boolean requestIdle;
  private boolean requestIntakeAlgae;
  private boolean requestIntakeCoral;
  private boolean requestReleaseAlgae;
  private boolean requestReleaseCoral;
  private boolean requestEject;

  private boolean coralHeld = false;
  private boolean algaeHeld = false;

  private enum EndEffectorStates {
    IDLE,
    INTAKE_ALGAE,
    INTAKE_CORAL,
    HOLD_ALGAE,
    HOLD_CORAL,
    RELEASE_ALGAE,
    RELEASE_CORAL,
    EJECT
  }

  private EndEffectorStates state = EndEffectorStates.IDLE;

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("End Effector", inputs);
    Logger.recordOutput("End Effector/State", state.toString());
    Logger.recordOutput("End Effector/coralHeld", coralHeld);
    Logger.recordOutput("End Effector/algaeHeld", algaeHeld);

    switch (state) {
      case IDLE:
        io.stopEndEffectorMotor(IdleMode.kCoast);
        if (requestIntakeAlgae) {
          state = EndEffectorStates.INTAKE_ALGAE;
        } else if (requestIntakeCoral) {
          state = EndEffectorStates.INTAKE_CORAL;
        }
        break;
      case INTAKE_ALGAE:
        io.setEndEffectorMotorVoltage(Constants.EndEffector.algaeIntakeVolts);
        if (io.isAlgaeProximityDetected() || io.isCurrentDetectionPickupTriggered()) {
          state = EndEffectorStates.HOLD_ALGAE;
          algaeHeld = true;
        }
        if (requestIdle) {
          state = EndEffectorStates.IDLE;
          algaeHeld = false;
        }
        break;
      case INTAKE_CORAL:
        io.setEndEffectorMotorVoltage(Constants.EndEffector.coralIntakeVolts);
        if (io.isCoralProximityDetected() || io.isCurrentDetectionPickupTriggered()) {
          state = EndEffectorStates.HOLD_CORAL;
          coralHeld = true;
        }
        if (requestIdle) {
          state = EndEffectorStates.IDLE;
          coralHeld = false;
        }
        break;
      case HOLD_ALGAE:
        io.setEndEffectorMotorVoltage(Constants.EndEffector.algaeHoldVolts);
        if (requestReleaseAlgae) {
          state = EndEffectorStates.RELEASE_ALGAE;
        } else if (requestEject) {
          state = EndEffectorStates.EJECT;
        }
        break;
      case HOLD_CORAL:
        io.setEndEffectorMotorVoltage(Constants.EndEffector.coralHoldVolts);
        if (requestReleaseCoral) {
          state = EndEffectorStates.RELEASE_CORAL;
        } else if (requestEject) {
          state = EndEffectorStates.EJECT;
        }
        break;
      case RELEASE_ALGAE:
        io.setEndEffectorMotorVoltage(Constants.EndEffector.algaeReleaseVolts);
        if (io.isCurrentDetectionReleaseTriggered() || !io.isAlgaeProximityDetected()) {
          state = EndEffectorStates.IDLE;
          algaeHeld = false;
        }
        break;
      case RELEASE_CORAL:
        io.setEndEffectorMotorVoltage(Constants.EndEffector.coralReleaseVolts);
        if (io.isCurrentDetectionReleaseTriggered()
            || (!io.isCoralProximityDetected() && !io.isAlgaeProximityDetected())) {
          state = EndEffectorStates.IDLE;
          coralHeld = false;
        }
        break;
      case EJECT:
        io.setEndEffectorMotorVoltage(Constants.EndEffector.ejectVolts);
        if (io.isCurrentDetectionReleaseTriggered()
            || (!io.isCoralProximityDetected() && !io.isAlgaeProximityDetected())) {
          state = EndEffectorStates.IDLE;
          coralHeld = false;
          algaeHeld = false;
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

  public void releaseCoral() {
    unsetAllRequests();
    requestReleaseCoral = true;
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
    io.stopEndEffectorMotor(mode);
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestIntakeAlgae = false;
    requestIntakeCoral = false;
    requestReleaseAlgae = false;
    requestReleaseCoral = false;
    requestEject = false;
  }
}
