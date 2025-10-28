package frc.robot.autonomous;

import frc.robot.autonomous.modes.DoNothing;
import frc.robot.autonomous.modes.Leave;
import frc.robot.autonomous.modes.OneCoralOneAlgaeCenter;
import frc.robot.autonomous.modes.OneCoralOneAlgaeEjectCenter;
import frc.robot.autonomous.modes.OneCoralTwoAlgaeCenter;
import frc.robot.autonomous.modes.OneCoralTwoAlgaeLeft;
import frc.robot.autonomous.modes.TestLeave;
import frc.robot.autonomous.modes.ThreeCoralLeft;
import frc.robot.autonomous.modes.ThreeCoralRight;
import frc.robot.autonomous.modes.TwoCoralOneAlgaeLeft;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OrangeSequentialCommandGroup;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousSelector {

  private LoggedDashboardChooser<OrangeSequentialCommandGroup> autonomousSelector =
      new LoggedDashboardChooser<OrangeSequentialCommandGroup>("Autonomous");

  public AutonomousSelector(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    autonomousSelector.addDefaultOption("DO_NOTHING", new DoNothing(superstructure));
    autonomousSelector.addOption("LEAVE", new Leave(drive, superstructure));
    autonomousSelector.addOption(
        "ONE_CORAL_TWO_ALGAE_CENTER", new OneCoralTwoAlgaeCenter(drive, superstructure));
    autonomousSelector.addOption(
        "ONE_CORAL_TWO_ALGAE_LEFT", new OneCoralTwoAlgaeLeft(drive, superstructure));
    autonomousSelector.addOption(
        "THREE_CORAL_LEFT",
        new ThreeCoralLeft(drive, superstructure, intakeSuperstructure, vision));
    autonomousSelector.addOption(
        "THREE_CORAL_RIGHT",
        new ThreeCoralRight(drive, superstructure, intakeSuperstructure, vision));
    autonomousSelector.addOption(
        "ONE_CORAL_ONE_ALGAE_CENTER", new OneCoralOneAlgaeCenter(drive, superstructure));
    autonomousSelector.addOption(
        "TWO_CORAL_ONE_ALGAE_LEFT",
        new TwoCoralOneAlgaeLeft(drive, superstructure, intakeSuperstructure, vision));
    autonomousSelector.addOption(
        "ONE_CORAL_ONE_ALGAE_EJECT_CENTER",
        new OneCoralOneAlgaeEjectCenter(drive, superstructure, intakeSuperstructure, vision));
    if (Constants.wantDriveTestAutos) {
      autonomousSelector.addOption("TEST_LEAVE", new TestLeave(drive, superstructure));
    }
  }

  public OrangeSequentialCommandGroup get() {
    return autonomousSelector.get();
  }
}
