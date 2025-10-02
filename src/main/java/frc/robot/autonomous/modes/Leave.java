package frc.robot.autonomous.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class Leave extends SequentialCommandGroup {
  public Leave(Drive drive) {
    setName("LEAVE");
    addRequirements(drive);
    /*addCommands(
        new AutoPoseReset(swerve, Robot.Leave.getStartingHolonomicPose().get().getTranslation()),
    AutoBuilder.followPath(Robot.Leave)
    );*/
  }
}
