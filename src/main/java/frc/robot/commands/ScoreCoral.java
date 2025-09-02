package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmIO;

public class ScoreCoral extends Command {
  private ArmIO armio;
  public ScoreCoral() {
   armio.setSpeed(50,50);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false; // TODO have actual logic
  }

  @Override
  public void end(boolean interrupted) {}
}
