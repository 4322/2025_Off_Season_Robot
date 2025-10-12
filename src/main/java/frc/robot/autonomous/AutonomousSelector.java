package frc.robot.autonomous;

import static frc.robot.RobotContainer.autonomousSelector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.modes.TestAuto;
import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousSelector {

  private LoggedDashboardChooser<Command> autonomousSelector =
      new LoggedDashboardChooser<Command>("Autonomous");

  public AutonomousSelector(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    /*
        autonomousSelector.addOption("DO_NOTHING", new DoNothing(superstructure));
        autonomousSelector.addOption("LEAVE", new Leave(drive, superstructure));
        autonomousSelector.addOption(
            "THREE_CORAL_LEFT",
            new ThreeCoralLeft(drive, superstructure, intakeSuperstructure, vision));
        autonomousSelector.addOption(
            "THREE_CORAL_RIGHT",
            new ThreeCoralRight(drive, superstructure, intakeSuperstructure, vision));
        autonomousSelector.addOption(
            "ONE_CORAL_ONE_ALGAE_CENTER",
            new OneCoralOneAlgaeCenter(drive, superstructure, intakeSuperstructure, vision));
        autonomousSelector.addOption(
            "TWO_CORAL_ONE_ALGAE_LEFT",
            new TwoCoralOneAlgaeLeft(drive, superstructure, intakeSuperstructure, vision));
        autonomousSelector.addOption(
            "ONE_CORAL_ONE_ALGAE_EJECT_CENTER",
            new OneCoralOneAlgaeEjectCenter(drive, superstructure, intakeSuperstructure, vision));
    */
    autonomousSelector.addDefaultOption(
        "TestAuto", new TestAuto(superstructure, drive, intakeSuperstructure, vision));
  }

  public Command get() {
    return autonomousSelector.get();
  }
}
