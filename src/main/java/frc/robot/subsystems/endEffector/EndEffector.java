package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.Logger;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
  private EndEffectorIO io;
  private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  private boolean requestIdle;
  private boolean requestIntakeAlgae;
  private boolean requestIntakeCoral;
  private boolean requestReleaseAlgae;
  private boolean requestReleaseCoral;

  private boolean coralHeld = false;
  private boolean algaeHeld = false;

  private enum EndEffectorStates {
    IDLE,
    INTAKE_ALGAE,
    INTAKE_CORAL,
    HOLD_ALGAE,
    HOLD_CORAL,
    RELEASE_ALGAE,
    RELEASE_CORAL
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
  }
}
