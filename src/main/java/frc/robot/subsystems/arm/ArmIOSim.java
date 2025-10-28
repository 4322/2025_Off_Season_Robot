package frc.robot.subsystems.arm;

public class ArmIOSim implements ArmIO {

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
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armConnected = true;

    double prevPos = position;
    simPos();
    simVolts();
    double velocity = (position - prevPos) * 50;

    inputs.requestedPosDeg = requestedPosition;
    inputs.PositionDegrees = position;
    inputs.voltage = voltage;
    inputs.velocityDegSec = velocity;
  }

  @Override
  public void setVoltage(double voltage) {
    requestedVoltage = voltage;
    requestedPosition = undefinedPosition;
  }

  @Override
  public void setHomePosition(double degrees) {
    stopArmMotor();
    this.position = 0;
  }

  @Override
  public void requestSlowPosition(double degrees) {
    requestedPosition = degrees;
    rate = slowRate;
    requestedVoltage = undefinedVoltage;
  }

  @Override
  public void requestPositionCoral(double degrees) {
    requestedPosition = degrees;
    rate = fastRate;
    requestedVoltage = undefinedVoltage;
  }

  @Override
  public void requestPositionAlgae(double degrees) {
    requestedPosition = degrees;
    rate = slowRate;
    requestedVoltage = undefinedVoltage;
  }

  @Override
  public void stopArmMotor() {
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
    position += 100 * voltage / 12.0 / 50.0;
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
