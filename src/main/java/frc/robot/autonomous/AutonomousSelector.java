package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.modes.Leave;
import frc.robot.autonomous.modes.OneCoralOneAlgaeCenter;
import frc.robot.autonomous.modes.ThreeCoralLeft;
import frc.robot.autonomous.modes.ThreeCoralRight;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousSelector {

  private LoggedDashboardChooser<SequentialCommandGroup> autonomousSelector =
      new LoggedDashboardChooser<SequentialCommandGroup>("Autonomous");

  public AutonomousSelector(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    autonomousSelector.addDefaultOption("DO_NOTHING", new SequentialCommandGroup());
    autonomousSelector.addOption("LEAVE", new Leave(drive));
    autonomousSelector.addOption(
        "THREE_CORAL_LEFT",
        new ThreeCoralLeft(drive, superstructure, intakeSuperstructure, vision));
    autonomousSelector.addOption(
        "THREE_CORAL_RIGHT",
        new ThreeCoralRight(drive, superstructure, intakeSuperstructure, vision));
    autonomousSelector.addOption(
        "ONE_CORAL_ONE_ALGAE_CENTER",
        new OneCoralOneAlgaeCenter(drive, superstructure, intakeSuperstructure, vision));
  }

  public SequentialCommandGroup get() {
    return autonomousSelector.get();
  }
}
