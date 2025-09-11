package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import static frc.robot.RobotContainer.driver;

public class AlgaeIntakeGround extends Command {
  private Superstructure superstructure;

  public AlgaeIntakeGround(Superstructure superstructure) {
    this.superstructure = superstructure;

    addRequirements(superstructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    superstructure.requestIntakeAlgaeFloor();

    if (!driver.a().getAsBoolean() || superstructure.isAlgaeHeld()) {
       cancel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!superstructure.isAlgaeHeld()) {
        superstructure.requestIdle();
    } // Superstructure automatically handles the transition back to algae idle
  }
}
