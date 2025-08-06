package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    private boolean armIdle;
    private ArmState currentState = ArmState.STARTING_CONFIG;
  

    public enum ArmState {
        STARTING_CONFIG,
        HOMING,
        REQUEST_SETPOINT,
        JIGGLE
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case STARTING_CONFIG:
                // Handle starting config logic
                break;
            case HOMING:
                // Handle homing logic
                break;
            case REQUEST_SETPOINT:
                // Handle request setpoint logic
                break;
            case JIGGLE:
                // Handle jiggle logic
                break;
        }
    }

    public void armIdle() {
        unsetAllRequests();
        armIdle = true;
    }
    private void unsetAllRequests() {

    }
}

