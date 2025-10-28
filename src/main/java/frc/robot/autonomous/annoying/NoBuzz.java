package frc.robot.autonomous.annoying;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.util.OrangeSequentialCommandGroup;

public class NoBuzz extends OrangeSequentialCommandGroup {

  public NoBuzz() {
    setName("NO_BUZZ");
    driver.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
  }
}
