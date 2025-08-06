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
                arm.setHome();
                // TODO
                break;
            case IDLE:
                arm.idle();
                // TODO
                break;
            case EJECT:
                arm.eject();
                // TODO
                break;
            case ALGAE_IDLE:
                arm.algaeHold()

                // TODO
                break;
            case ALGAE_PRESCORE:
                arm.scoreAlgae()
                // TODO
                break;
            case ALGAE_SCORE:
                arm.algaeReef(Level requestedAlgaeLevel)

                // TODO
                break;
            case INTAKE_ALGAE_FLOOR:
                arm.algaeGround()

                // TODO
                break;
            case DESCORE_ALGAE:
                // TODO
                break;
            case END_EFFECTOR_CORAL_PICKUP:
                // TODO
                break;
            case CORAL_HELD:
             arm.idle()

                // TODO
                break;
            case PRESCORE_CORAL:
                arm.prescoreCoral(Level requestedCoralLevel)
                // TODO
                break;
            case SCORE_CORAL:
             arm.scoreCoral(Level requestedCoralLevel)

                // TODO
                break;
            case SAFE_SCORE_ALGAE_RETRACT:
                arm.safeBargeRetract()
                arm.atSetpoint()

                // TODO
                break;
            case PRECLIMB:
            arm.climbing()

                // TODO
                break;
            case CLIMB:
                // TODO
                break;
        }
    }
}
