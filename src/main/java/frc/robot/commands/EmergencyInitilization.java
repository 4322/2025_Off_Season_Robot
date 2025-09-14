package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class EmergencyInitilization extends Command {
  private final Superstructure superstructure;
  public boolean isSlow = false;

  public EmergencyInitilization(Superstructure superstructure) {
    this.superstructure = superstructure;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
   
  }

  @Override
  public void execute() {

    
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
