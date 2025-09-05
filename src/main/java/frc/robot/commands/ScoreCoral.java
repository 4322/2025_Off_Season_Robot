package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.drive.Drive;

public class ScoreCoral extends Command {
  private ArmIO armio;
  private Arm arm;
  private ElevatorIO elevatorio;
  private Elevator elevator;
  private Superstructure.Level level;
  private Superstructure superstructure;

  public ScoreCoral() {}

  @Override
  public void initialize() {
    superstructure.requestPrescoreCoral(level);
  }

  @Override
  public void execute() {
    if (superstructure.getState() == Superstructure.Superstates.PRESCORE_CORAL) {
      superstructure.getReefStatus();
      if (true // TODO replace with actual logic
          ) {
        superstructure.requestScoreCoral(level);
      }
      if (superstructure.isAutoOperationMode()) {
      } else {

      }
    }

    arm.scoreCoral(level);
    elevator.scoreCoral(level);
  }

  @Override
  public boolean isFinished() {
    return false; // TODO have actual logic
  }

  @Override
  public void end(boolean interrupted) {}
}
