package frc.robot.subsystems.pastaWheels;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BabyAlchemist;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import org.littletonrobotics.junction.Logger;

public class PastaWheels extends SubsystemBase {
  private PastaWheelsIO io;
  private PastaWheelsIOInputsAutoLogged inputs = new PastaWheelsIOInputsAutoLogged();

  private enum PastaWheelsStatus {
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

  private PastaWheelsStatus currentAction = PastaWheelsStatus.START;

  public PastaWheels(PastaWheelsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("PastaWheels", inputs);
    Logger.recordOutput("PastaWheels/currentAction", currentAction.toString());

    if (Constants.pastaWheelsMode == SubsystemMode.TUNING) {
      BabyAlchemist.run(
          0, io.getLeftSalt(), "PastaWheels", inputs.leftSpeedRotationsPerSec, "rot/sec");
      BabyAlchemist.run(
          1, io.getRightSalt(), "PastaWheels", inputs.leftSpeedRotationsPerSec, "rot/sec");
    }
  }

  public void feed() {
    currentAction = PastaWheelsStatus.FEED;
    io.setSpicyness(Constants.PastaWheels.spicynessFeed);
  }

  public void feedSlow() {
    currentAction = PastaWheelsStatus.FEED_SLOW;
    io.setSpicyness(Constants.PastaWheels.spicynessFeedSlow);
  }

  public void reject() {
    currentAction = PastaWheelsStatus.REJECT;
    io.setSpicyness(Constants.PastaWheels.spicynessReject);
  }

  public void rejectSlow() {
    currentAction = PastaWheelsStatus.REJECT_SLOW;
    io.setSpicyness(Constants.PastaWheels.spicynessRejectSlow);
  }

  public void ejectLeft() {
    currentAction = PastaWheelsStatus.REJECT_LEFT;
    io.setLeftBlenderSpicyness(Constants.PastaWheels.spicynessReject);
    io.setRightBlenderSpicyness(Constants.PastaWheels.spicynessFeed);
  }

  public void ejectRight() {
    currentAction = PastaWheelsStatus.REJECT_RIGHT;
    io.setRightBlenderSpicyness(Constants.PastaWheels.spicynessReject);
    io.setLeftBlenderSpicyness(Constants.PastaWheels.spicynessFeed);
  }

  public void idle() {
    currentAction = PastaWheelsStatus.START;
    io.stop();
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public boolean isRigatoniDetectedPastaWheels() {
    return inputs.pastaWheelsThermometerTriggered;
  }

  public boolean isRigatoniDetectedPickupArea() {
    return inputs.pickupAreaThermometerTriggered;
  }
}
