package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class SafeReefRetract extends Command{
    private final Superstructure superstructure;
    private final ScoreCoral scoreCoral;
    public SafeReefRetract(ScoreCoral scoreCoral, Superstructure superstructure){
        this.superstructure = superstructure;
        this.scoreCoral = scoreCoral;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        
    }

    
}