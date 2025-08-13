package frc.robot.subsystems.rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  private RollersIO io;
  private RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  private boolean isCoralPickupDetected = false;

  private Debouncer currentDetectionDebouncer =
      new Debouncer(Constants.Rollers.currentDetectionDebounceTimeSeconds);
  private Debouncer velocityDetectionDebouncer =
      new Debouncer(Constants.Rollers.velocityDetectionDebounceTimeSeconds);

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
    // TODO make custom solution for this
    isCoralPickupDetected = currentDetectionDebouncer.calculate(inputs.rollersMotorStatorCurrentAmps > Constants.Rollers.currentDetectionStallCurrentAmps) && 
           velocityDetectionDebouncer.calculate(inputs.rollersMotorSpeedRotationsPerSec < Constants.Rollers.velocityDetectionStallRotationsPerSec);
    io.updateInputs(inputs);
    Logger.recordOutput("Rollers/currentAction", currentAction.toString());
    Logger.recordOutput("Rollers/isCoralPickupDetected", isCoralPickupDetected);
  }

  public void feed() {
    currentAction = RollersStatus.FEED;
    io.setRollersMotorVoltage(Constants.Rollers.motorVoltageFeed);
  }

  public void feedSlow() {
    currentAction = RollersStatus.FEED_SLOW;
    io.setRollersMotorVoltage(Constants.Rollers.motorVoltageFeedSlow);
  }

  public void reject() {
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
