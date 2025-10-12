package frc.robot.autonomous.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.OrangeSequentialCommandGroup;

public class DoNothing extends OrangeSequentialCommandGroup {
  public DoNothing(Superstructure superstructure) {
    setName("DO_NOTHING");
    addCommands(
        new InstantCommand(
            () -> superstructure.requestOperationMode(Superstructure.OperationMode.AUTO)));
  }
}
