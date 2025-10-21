package frc.robot.autonomous;

import frc.robot.autonomous.modes.DoNothing;
import frc.robot.autonomous.modes.Leave;
import frc.robot.autonomous.modes.OneRigatoniOneMeatballCenter;
import frc.robot.autonomous.modes.OneRigatoniOneMeatballEjectCenter;
import frc.robot.autonomous.modes.OneRigatoniTwoMeatballCenter;
import frc.robot.autonomous.modes.OneRigatoniTwoMeatballLeft;
import frc.robot.autonomous.modes.ThreeRigatoniLeft;
import frc.robot.autonomous.modes.ThreeRigatoniRight;
import frc.robot.autonomous.modes.TwoRigatoniOneMeatballLeft;
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
        "ONE_RIGATONI_TWO_MEATBALL_CENTER", new OneRigatoniTwoMeatballCenter(drive, superstructure));
    autonomousSelector.addOption(
        "ONE_RIGATONI_TWO_MEATBALL_LEFT", new OneRigatoniTwoMeatballLeft(drive, superstructure));
    autonomousSelector.addOption(
        "THREE_RIGATONI_LEFT",
        new ThreeRigatoniLeft(drive, superstructure, intakeSuperstructure, vision));
    autonomousSelector.addOption(
        "THREE_RIGATONI_RIGHT",
        new ThreeRigatoniRight(drive, superstructure, intakeSuperstructure, vision));
    autonomousSelector.addOption(
        "ONE_RIGATONI_ONE_MEATBALL_CENTER", new OneRigatoniOneMeatballCenter(drive, superstructure));
    autonomousSelector.addOption(
        "TWO_RIGATONI_ONE_MEATBALL_LEFT",
        new TwoRigatoniOneMeatballLeft(drive, superstructure, intakeSuperstructure, vision));
    autonomousSelector.addOption(
        "ONE_RIGATONI_ONE_MEATBALL_EJECT_CENTER",
        new OneRigatoniOneMeatballEjectCenter(drive, superstructure, intakeSuperstructure, vision));
  }

  public OrangeSequentialCommandGroup get() {
    return autonomousSelector.get();
  }
}
