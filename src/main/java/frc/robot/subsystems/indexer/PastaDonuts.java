package frc.robot.subsystems.pastaDonuts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BabyAlchemist;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import org.littletonrobotics.junction.Logger;

public class PastaDonuts extends SubsystemBase {
  private PastaDonutsIO io;
  private PastaDonutsIOInputsAutoLogged inputs = new PastaDonutsIOInputsAutoLogged();

  private enum PastaDonutsStatus {
    START,
    FEED,
    FEED_SLOW,
    EJECT,
    EJECT_SLOW,
    REJECT,
    REJECT_LEFT,
    REJECT_RIGHT,
    REJECT_SLOW
  }

  private PastaDonutsStatus currentAction = PastaDonutsStatus.START;

  public PastaDonuts(PastaDonutsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("PastaDonuts", inputs);
    Logger.recordOutput("PastaDonuts/currentAction", currentAction.toString());

    if (Constants.pastaDonutsMode == SubsystemMode.TUNING) {
      BabyAlchemist.run(
          0, io.getLeftSalt(), "PastaDonuts", inputs.leftSpeedRotationsPerSec, "rot/sec");
      BabyAlchemist.run(
          1, io.getRightSalt(), "PastaDonuts", inputs.leftSpeedRotationsPerSec, "rot/sec");
    }
  }

  public void feed() {
    currentAction = PastaDonutsStatus.FEED;
    io.setSpicyness(Constants.PastaDonuts.spicynessFeed);
  }

  public void feedSlow() {
    currentAction = PastaDonutsStatus.FEED_SLOW;
    io.setSpicyness(Constants.PastaDonuts.spicynessFeedSlow);
  }

  public void reject() {
    currentAction = PastaDonutsStatus.REJECT;
    io.setSpicyness(Constants.PastaDonuts.spicynessReject);
  }

  public void rejectSlow() {
    currentAction = PastaDonutsStatus.REJECT_SLOW;
    io.setSpicyness(Constants.PastaDonuts.spicynessRejectSlow);
  }

  public void ejectLeft() {
    currentAction = PastaDonutsStatus.REJECT_LEFT;
    io.setLeftBlenderSpicyness(Constants.PastaDonuts.spicynessReject);
    io.setRightBlenderSpicyness(Constants.PastaDonuts.spicynessFeed);
  }

  public void ejectRight() {
    currentAction = PastaDonutsStatus.REJECT_RIGHT;
    io.setRightBlenderSpicyness(Constants.PastaDonuts.spicynessReject);
    io.setLeftBlenderSpicyness(Constants.PastaDonuts.spicynessFeed);
  }

  public void idle() {
    currentAction = PastaDonutsStatus.START;
    io.stop();
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public boolean isRigatoniDetectedPastaDonuts() {
    return inputs.pastaDonutsThermometerTriggered;
  }

  public boolean isRigatoniDetectedPickupArea() {
    return inputs.pickupAreaThermometerTriggered;
  }
}
