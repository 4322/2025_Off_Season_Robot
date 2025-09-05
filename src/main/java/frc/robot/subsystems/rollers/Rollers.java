package frc.robot.subsystems.rollers;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
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
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.recordOutput("Rollers/currentAction", currentAction.toString());

    currentDetectionTriggered =
        currentDetectionDebouncer.calculate(inputs.rollersMotorStatorCurrentAmps);
    velocityDetectionTriggered =
        velocityDetectionDebouncer.calculate(inputs.rollersMotorSpeedRotationsPerSec);
    isCoralPickupDetected = currentDetectionTriggered && velocityDetectionTriggered;

    Logger.recordOutput("Rollers/isCoralPickupDetected", isCoralPickupDetected);
    Logger.recordOutput("Rollers/currentDetectionTriggered", currentDetectionTriggered);
    Logger.recordOutput("Rollers/velocityDetectionTriggered", velocityDetectionTriggered);
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

  public void idle() {
    currentAction = RollersStatus.IDLE;
    io.stopRollersMotor(IdleMode.kCoast);
  }

  public boolean isCoralPickupDetected() {
    return isCoralPickupDetected;
  }
}
