package frc.robot.autonomous.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Superstructure;

public class DoNothing extends SequentialCommandGroup {
  public DoNothing(Superstructure superstructure) {
    setName("DO_NOTHING");
    addCommands(
        new InstantCommand(
            () -> superstructure.requestOperationMode(Superstructure.OperationMode.AUTO)));
  }
}
