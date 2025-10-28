package frc.robot.autonomous.annoying;

import frc.robot.subsystems.IntakeSuperstructure;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OrangeSequentialCommandGroup;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Selector {

  private LoggedDashboardChooser<OrangeSequentialCommandGroup> selector =
      new LoggedDashboardChooser<OrangeSequentialCommandGroup>("Buzzing");

  public Selector(
      Drive drive,
      Superstructure superstructure,
      IntakeSuperstructure intakeSuperstructure,
      Vision vision) {
    selector.addOption("BUZZ", new Buzz());

    selector.addDefaultOption("NO_BUZZ", new NoBuzz());
  }

  public OrangeSequentialCommandGroup get() {
    return selector.get();
  }
}
