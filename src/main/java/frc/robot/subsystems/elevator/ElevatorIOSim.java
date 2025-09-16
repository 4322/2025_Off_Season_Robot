package frc.robot.subsystems.elevator;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;

public class ElevatorIOSim implements ElevatorIO {
  private double requestedVoltage = 0;
  private double requestedPosition = 0;

  private double voltage = 0;
  private double position = 0;
  private double undefinedVoltage = -20;
  private double undefinedPosition = -1;

  private double slowRate = 0.02;
  private double fastRate = 0.05;
  private double rate;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leaderElevatorMotorConnected = true;
    inputs.followerElevatorMotorConnected = true;

    double prevPos = position;
    simPos();
    simVolts();
    double velocity = (position - prevPos) * 50;

    inputs.requestedPosMeters = requestedPosition;
    inputs.leaderMotorheightMeters = position;
    inputs.followerMotorheightMeters = position;

    inputs.leaderMotorVoltage = voltage;
    inputs.followerMotorVoltage = voltage;

    inputs.followerMotorVelocityMetersPerSecond = velocity;
    inputs.leaderMotorVelocityMetersPerSecond = velocity;
  }

  @Override
  public void setVoltage(double voltage) {
    requestedVoltage = voltage;
    requestedPosition = undefinedPosition;
  }

  @Override
  public void setPosition(double position) {
    stop(IdleMode.kBrake);
    this.position = position;
  }

  @Override
  public void requestSlowHeightMeters(double heightMeters) {
    requestedPosition = heightMeters;
    rate = slowRate;
    requestedVoltage = undefinedVoltage;
  }

  @Override
  public void requestHeightMeters(double heightMeters) {
    requestedPosition = heightMeters;
    rate = fastRate;
    requestedVoltage = undefinedVoltage;
  }

  @Override
  public void stop(IdleMode idleMode) {
    requestedPosition = undefinedPosition;
    requestedVoltage = undefinedVoltage;
  }

  private void simVolts() {
    if (requestedVoltage == undefinedVoltage) {
      voltage = 0;
    } else if (voltage < requestedVoltage) {
      voltage += (requestedVoltage - voltage) * fastRate;
    } else {
      voltage -= (voltage - requestedVoltage) * fastRate;
    }
    position += voltage / 12.0 / 50.0;
  }

  private void simPos() {
    if (requestedPosition == undefinedPosition) {
      return;
    }
    if (position < requestedPosition) {
      position += (requestedPosition - position) * rate;
    } else {
      position -= (position - requestedPosition) * rate;
    }
  }
}
