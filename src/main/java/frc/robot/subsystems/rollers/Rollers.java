package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {

    private boolean coralDetected;

    public Rollers() {

    }

    @Override
    public void periodic() {
        // Checks current in here, sets coralDetected depending on threshold
    }

    public void feed() {

    }
    
    public void reject() {

    }
    
    public void feedSlow() {

    }

    public void rejectSlow() {

    }

    public void eject() {

    }

    public boolean isCoralDetected() {
        return coralDetected;
    }
}
