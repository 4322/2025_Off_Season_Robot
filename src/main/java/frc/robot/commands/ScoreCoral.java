package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
public class ScoreCoral extends Command {
  private ArmIO armio;
  private Arm arm;
  private ElevatorIO elevatorio;
  private Elevator elevator;
  private Superstructure.Level level;
  public ScoreCoral() {
   
  }

  @Override
  public void initialize() {
    armio.setSpeed(50,50);
    elevatorio.setSpeed(20,20);
  }

  @Override
  public void execute() {
    arm.scoreCoral();
    elevator.scoreCoral(level);
    
  }

  @Override
  public boolean isFinished() {
    return false; // TODO have actual logic
  }

  @Override
  public void end(boolean interrupted) {}
}
