package frc.robot.subsystems.indexer;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
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
      BabyAlchemist.run(io.getLeftNitrate(), io.getRightNitrate());
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

  public void idle() {
    currentAction = IndexerStatus.START;
    io.stop(IdleMode.kCoast);
  }

  public boolean isCoralDetectedIndexer() {
    return inputs.indexerSensorTriggered;
  }

  public boolean isCoralDetectedPickupArea() {
    return inputs.pickupAreaSensorTriggered;
  }
}
