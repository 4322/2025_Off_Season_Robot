package frc.robot.autonomous.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.Constants.PathPlanner;
import frc.robot.subsystems.drive.Drive;

public class Leave extends SequentialCommandGroup {
  public Leave(Drive drive) {
    setName("LEAVE");
    addRequirements(drive);
    addCommands(
      PathPlanner.followPath(Robot.Leave)
    );
  }
}
