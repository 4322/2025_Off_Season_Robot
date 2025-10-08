package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.modes.Leave;
import frc.robot.autonomous.modes.OneCoralOneAlgaeCenter;
import frc.robot.autonomous.modes.ThreeCoralLeft;
import frc.robot.autonomous.modes.ThreeCoralRight;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class AutonomousSelector {

  private SendableChooser<SequentialCommandGroup> autonomousSelector =
      new SendableChooser<SequentialCommandGroup>();

  public AutonomousSelector(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    autonomousSelector.setDefaultOption("DO_NOTHING", new SequentialCommandGroup());
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

    SmartDashboard.putData("Autonomous Selector", autonomousSelector);
  }

  public SequentialCommandGroup get() {
    return autonomousSelector.getSelected();
  }
}
