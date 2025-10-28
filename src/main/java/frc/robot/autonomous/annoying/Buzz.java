package frc.robot.autonomous.annoying;

import static frc.robot.RobotContainer.driver;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.OrangeSequentialCommandGroup;

public class Buzz extends OrangeSequentialCommandGroup {
  Timer batteryLowStopTimer = new Timer();

  public Buzz() {

    setName("BUZZ");
    driver.setRumble(GenericHID.RumbleType.kBothRumble, 20.0);

    //   if (RobotController.getBatteryVoltage() <= 12.00
    //     && (superstructure.getState() == Superstructure.Superstates.IDLE
    //         || superstructure.getState() == Superstructure.Superstates.CORAL_HELD)
    //     && intakeSuperstructure.getIntakeSuperstate()
    //         == IntakeSuperstructure.IntakeSuperstates.RETRACT_IDLE
    //     && !DriverStation.isFMSAttached()) {
    //   driver.setRumble(GenericHID.RumbleType.kBothRumble, 10.0);
    //   batteryLowStopTimer.start();
    //   if (batteryLowStopTimer.hasElapsed(5)) {
    //     driver.setRumble(GenericHID.RumbleType.kBothRumble, 20.0);
    //     batteryLowStopTimer.stop();
    //     batteryLowStopTimer.reset();
    //     DriverStation.reportError(
    //         "Battery voltage is critically low (" + RobotController.getBatteryVoltage() + "V)",
    //         false);
    //   }
    // } else {
    //   batteryLowStopTimer.stop();
    //   batteryLowStopTimer.reset();
    // }

    // if (RobotController.getBatteryVoltage() > 12.00) {
    //   driver.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    // }
  }
}
