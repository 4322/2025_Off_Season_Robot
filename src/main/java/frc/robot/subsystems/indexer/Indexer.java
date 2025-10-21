package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BabyAlchemist;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO io;
  private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private enum IndexerStatus {
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

  private IndexerStatus currentAction = IndexerStatus.START;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    Logger.recordOutput("Indexer/currentAction", currentAction.toString());

    if (Constants.indexerMode == SubsystemMode.TUNING) {
      BabyAlchemist.run(
          0, io.getLeftNitrate(), "Indexer", inputs.leftSpeedRotationsPerSec, "rot/sec");
      BabyAlchemist.run(
          1, io.getRightNitrate(), "Indexer", inputs.leftSpeedRotationsPerSec, "rot/sec");
    }
  }

  public void feed() {
    currentAction = IndexerStatus.FEED;
    io.setVoltage(Constants.Indexer.voltageFeed);
  }

  public void feedSlow() {
    currentAction = IndexerStatus.FEED_SLOW;
    io.setVoltage(Constants.Indexer.voltageFeedSlow);
  }

  public void reject() {
    currentAction = IndexerStatus.REJECT;
    io.setVoltage(Constants.Indexer.voltageReject);
  }

  public void rejectSlow() {
    currentAction = IndexerStatus.REJECT_SLOW;
    io.setVoltage(Constants.Indexer.voltageRejectSlow);
  }

  public void ejectLeft() {
    currentAction = IndexerStatus.REJECT_LEFT;
    io.setLeftMotorVoltage(Constants.Indexer.voltageReject);
    io.setRightMotorVoltage(Constants.Indexer.voltageFeed);
  }

  public void ejectRight() {
    currentAction = IndexerStatus.REJECT_RIGHT;
    io.setRightMotorVoltage(Constants.Indexer.voltageReject);
    io.setLeftMotorVoltage(Constants.Indexer.voltageFeed);
  }

  public void idle() {
    currentAction = IndexerStatus.START;
    io.stop();
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public boolean isCoralDetectedIndexer() {
    return inputs.indexerSensorTriggered;
  }

  public boolean isCoralDetectedPickupArea() {
    return inputs.pickupAreaSensorTriggered;
  }
}
