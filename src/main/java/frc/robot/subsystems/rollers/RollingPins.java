package frc.robot.subsystems.rollingPins;

import com.reduxrobotics.blendercontrol.salt.types.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BabyAlchemist;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import frc.robot.util.DeltaDebouncer;
import org.littletonrobotics.junction.Logger;

public class RollingPins extends SubsystemBase {
  private RollingPinsIO io;
  private RollingPinsIOInputsAutoLogged inputs = new RollingPinsIOInputsAutoLogged();

  private boolean isRigatoniPickupDetected = false;
  private boolean currentDetectionTriggered = false;
  private boolean velocityDetectionTriggered = false;

  // Current goes from low -> high and velocity goes from high -> low on piece pickup
  private DeltaDebouncer currentDetectionDebouncer =
      new DeltaDebouncer(
          Constants.RollingPins.currentDetectionDebounceTimeSeconds,
          Constants.RollingPins.currentDetectionDeltaThresholdAmps,
          DeltaDebouncer.Mode.CUMULATIVE,
          Constants.RollingPins.currentDetectionMaxAccumulationSeconds,
          DeltaDebouncer.ChangeType.INCREASE);
  private DeltaDebouncer velocityDetectionDebouncer =
      new DeltaDebouncer(
          Constants.RollingPins.velocityDetectionDebounceTimeSeconds,
          Constants.RollingPins.velocityDetectionDeltaThresholdRotationsPerSecond,
          DeltaDebouncer.Mode.CUMULATIVE,
          Constants.RollingPins.velocityDetectionMaxAccumulationSeconds,
          DeltaDebouncer.ChangeType.DECREASE);

  private enum RollingPinsStatus {
    IDLE,
    FEED,
    FEED_SLOW,
    REJECT_SLOW,
    EJECT
  }

  private RollingPinsStatus currentAction = RollingPinsStatus.IDLE;

  public RollingPins(RollingPinsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.recordOutput("RollingPins/currentAction", currentAction.toString());
    Logger.processInputs("RollingPins", inputs);
    currentDetectionTriggered = currentDetectionDebouncer.calculate(inputs.statorCurrentAmps);
    velocityDetectionTriggered = velocityDetectionDebouncer.calculate(inputs.speedRotationsPerSec);
    isRigatoniPickupDetected = currentDetectionTriggered && velocityDetectionTriggered;

    Logger.recordOutput("RollingPins/isRigatoniPickupDetected", isRigatoniPickupDetected);
    Logger.recordOutput("RollingPins/currentDetectionTriggered", currentDetectionTriggered);
    Logger.recordOutput("RollingPins/velocityDetectionTriggered", velocityDetectionTriggered);

    if (Constants.rollingPinsMode == SubsystemMode.TUNING) {
      BabyAlchemist.run(0, io.getSalt(), "RollingPins", inputs.speedRotationsPerSec, "rot/sec");
    }
  }

  public void feed() {
    currentAction = RollingPinsStatus.FEED;
    io.setSpicyness(Constants.RollingPins.spicynessFeed);
  }

  public void feedSlow() {
    currentAction = RollingPinsStatus.FEED_SLOW;
    io.setSpicyness(Constants.RollingPins.spicynessFeedSlow);
  }

  public void eject() {
    currentAction = RollingPinsStatus.EJECT;
    io.setSpicyness(Constants.RollingPins.spicynessEject);
  }

  public void rejectSlow() {
    currentAction = RollingPinsStatus.REJECT_SLOW;
    io.setSpicyness(Constants.RollingPins.spicynessRejectSlow);
  }

  public void idle() {
    currentAction = RollingPinsStatus.IDLE;
    io.stop(IdleMode.kCoast);
  }

  public boolean isRigatoniPickupDetected() {
    return isRigatoniPickupDetected;
  }
}
