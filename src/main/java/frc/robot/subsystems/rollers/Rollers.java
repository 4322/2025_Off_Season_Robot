package frc.robot.subsystems.rollers;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BabyAlchemist;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import frc.robot.util.DeltaDebouncer;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  private RollersIO io;
  private RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  private boolean isCoralPickupDetected = false;
  private boolean currentDetectionTriggered = false;
  private boolean velocityDetectionTriggered = false;
  private final Timer initTimer = new Timer();

  // Current goes from low -> high and velocity goes from high -> low on piece pickup
  private DeltaDebouncer currentDetectionDebouncer =
      new DeltaDebouncer(
          Constants.Rollers.currentDetectionDebounceTimeSeconds,
          Constants.Rollers.currentDetectionDeltaThresholdAmps,
          DeltaDebouncer.Mode.CUMULATIVE,
          Constants.Rollers.currentDetectionMaxAccumulationSeconds,
          DeltaDebouncer.ChangeType.INCREASE);
  private DeltaDebouncer velocityDetectionDebouncer =
      new DeltaDebouncer(
          Constants.Rollers.velocityDetectionDebounceTimeSeconds,
          Constants.Rollers.velocityDetectionDeltaThresholdRotationsPerSecond,
          DeltaDebouncer.Mode.CUMULATIVE,
          Constants.Rollers.velocityDetectionMaxAccumulationSeconds,
          DeltaDebouncer.ChangeType.DECREASE);

  private enum RollersStatus {
    IDLE,
    FEED,
    FEED_SLOW,
    REJECT,
    REJECT_SLOW,
    EJECT
  }

  private RollersStatus currentAction = RollersStatus.IDLE;

  public Rollers(RollersIO io) {
    this.io = io;
    initTimer.start();
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.recordOutput("Rollers/currentAction", currentAction.toString());
    Logger.processInputs("Rollers", inputs);
    currentDetectionTriggered = currentDetectionDebouncer.calculate(inputs.statorCurrentAmps);
    velocityDetectionTriggered = velocityDetectionDebouncer.calculate(inputs.speedRotationsPerSec);
    isCoralPickupDetected = currentDetectionTriggered && velocityDetectionTriggered;

    Logger.recordOutput("Rollers/isCoralPickupDetected", isCoralPickupDetected);
    Logger.recordOutput("Rollers/currentDetectionTriggered", currentDetectionTriggered);
    Logger.recordOutput("Rollers/velocityDetectionTriggered", velocityDetectionTriggered);

    if (Constants.rollersMode == SubsystemMode.TUNING && initTimer.hasElapsed(5)) {
      BabyAlchemist.run(0, io.getNitrate(), "Rollers", inputs.speedRotationsPerSec, "rot/sec");
    }
  }

  public void feed() {
    currentAction = RollersStatus.FEED;
    io.setVoltage(Constants.Rollers.voltageFeed);
  }

  public void feedSlow() {
    currentAction = RollersStatus.FEED_SLOW;
    io.setVoltage(Constants.Rollers.voltageFeedSlow);
  }

  public void reject() {
    currentAction = RollersStatus.REJECT;
    io.setVoltage(Constants.Rollers.voltageReject);
  }

  public void rejectSlow() {
    currentAction = RollersStatus.REJECT_SLOW;
    io.setVoltage(Constants.Rollers.voltageRejectSlow);
  }

  public void idle() {
    currentAction = RollersStatus.IDLE;
    io.stop(IdleMode.kCoast);
  }

  public boolean isCoralPickupDetected() {
    return isCoralPickupDetected;
  }
}
