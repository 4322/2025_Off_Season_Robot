package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.DeltaDebouncer;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  private RollersIO io;
  private RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  private boolean isCoralPickupDetected = false;

  // Current goes from low -> high and velocity goes from high -> low on piece pickup
  private DeltaDebouncer currentDetectionDebouncer =
      new DeltaDebouncer(
          Constants.Rollers.currentDetectionDebounceTimeSeconds,
          Constants.Rollers.CurrentDetectionDeltaThresholdAmps,
          DeltaDebouncer.Mode.CUMULATIVE,
          Constants.Rollers.CurrentDetectionMaxAccumulationSeconds,
          DeltaDebouncer.ChangeType.INCREASE);
  private DeltaDebouncer velocityDetectionDebouncer =
      new DeltaDebouncer(
          Constants.Rollers.velocityDetectionDebounceTimeSeconds,
          Constants.Rollers.VelocityDetectionDeltaThresholdRotationsPerSecond,
          DeltaDebouncer.Mode.CUMULATIVE,
          Constants.Rollers.VelocityDetectionMaxAccumulationSeconds,
          DeltaDebouncer.ChangeType.DECREASE);

  private enum RollersStatus {
    START,
    FEED,
    FEED_SLOW,
    REJECT,
    REJECT_SLOW,
    EJECT
  }

  private RollersStatus currentAction = RollersStatus.START;

  public Rollers(RollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.recordOutput("Rollers/currentAction", currentAction.toString());
    Logger.recordOutput(
        "Rollers/isCoralPickupDetected", isCoralPickupDetected); // TODO move after 52-54

    isCoralPickupDetected =
        currentDetectionDebouncer.calculate(inputs.rollersMotorStatorCurrentAmps)
            && velocityDetectionDebouncer.calculate(inputs.rollersMotorSpeedRotationsPerSec);
    // TODO log and set local variables to results of these ^
  }

  public void feed() {
    currentAction = RollersStatus.FEED;
    io.setRollersMotorVoltage(Constants.Rollers.motorVoltageFeed);
  }

  public void feedSlow() {
    currentAction = RollersStatus.FEED_SLOW;
    io.setRollersMotorVoltage(Constants.Rollers.motorVoltageFeedSlow);
  }

  public void reject() { // TODO check whether this is necessary
    currentAction = RollersStatus.REJECT;
    io.setRollersMotorVoltage(Constants.Rollers.motorVoltageReject);
  }

  public void rejectSlow() {
    currentAction = RollersStatus.REJECT_SLOW;
    io.setRollersMotorVoltage(Constants.Rollers.motorVoltageRejectSlow);
  }

  public void eject() {
    currentAction = RollersStatus.EJECT;
    io.setRollersMotorVoltage(Constants.Rollers.motorVoltageEject);
  }

  public boolean isCoralPickupDetected() {
    return isCoralPickupDetected;
  }
}
