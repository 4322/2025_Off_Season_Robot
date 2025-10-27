package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BabyTunerX;
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
    REJECT_SLOW,
    EJECT
  }

  private RollersStatus currentAction = RollersStatus.IDLE;

  public Rollers(RollersIO io) {
    this.io = io;
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

    if (Constants.rollersMode == SubsystemMode.TUNING) {
      BabyTunerX.run(0, io.getTalonFX(), "Rollers", inputs.speedRotationsPerSec, "rot/sec");
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

  public void eject() {
    currentAction = RollersStatus.EJECT;
    io.setVoltage(Constants.Rollers.voltageEject);
  }

  public void rejectSlow() {
    currentAction = RollersStatus.REJECT_SLOW;
    io.setVoltage(Constants.Rollers.voltageRejectSlow);
  }

  public void idle() {
    currentAction = RollersStatus.IDLE;
    io.stop();
  }

  public boolean isCoralPickupDetected() {
    return isCoralPickupDetected;
  }
}
