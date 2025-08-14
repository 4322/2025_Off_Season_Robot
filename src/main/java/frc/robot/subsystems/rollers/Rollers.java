package frc.robot.subsystems.rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.DeltaDebouncer;

import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  private RollersIO io;
  private RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  private boolean isCoralPickupDetected = false;

  private DeltaDebouncer currentDetectionDebouncer =
      new DeltaDebouncer(Constants.Rollers.currentDetectionDebounceTimeSeconds,Constants.Rollers.CurrentDetectionDeltaThresholdAmps, DeltaDebouncer.Mode.CUMULATIVE, Constants.Rollers.CurrentDetectionMaxAccumulationSeconds);
  private DeltaDebouncer velocityDetectionDebouncer =
      new DeltaDebouncer(Constants.Rollers.velocityDetectionDebounceTimeSeconds, Constants.Rollers.VelocityDetectionDeltaThresholdRotationsPerSecond, DeltaDebouncer.Mode.CUMULATIVE, Constants.Rollers.VelocityDetectionMaxAccumulationSeconds);
  
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
    isCoralPickupDetected = currentDetectionDebouncer.calculate(inputs.rollersMotorStatorCurrentAmps) && 
        velocityDetectionDebouncer.calculate(inputs.rollersMotorSpeedRotationsPerSec);
    
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
