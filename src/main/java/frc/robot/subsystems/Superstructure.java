package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
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

  SuperstructureState currentState = SuperstructureState.START;

  public void handleState() {
    switch (currentState) {
      case START:
        break;
      case IDLE:

        // TODO
        break;
      case EJECT:

        // TODO
        break;
      case ALGAE_IDLE:

        // TODO
        break;
      case ALGAE_PRESCORE:

        // TODO
        break;
      case ALGAE_SCORE:

        // TODO
        break;
      case INTAKE_ALGAE_FLOOR:

        // TODO
        break;
      case DESCORE_ALGAE:
        // TODO
        break;
      case END_EFFECTOR_CORAL_PICKUP:
        // TODO
        break;
      case CORAL_HELD:

        // TODO
        break;
      case PRESCORE_CORAL:

        // TODO
        break;
      case SCORE_CORAL:

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
}
