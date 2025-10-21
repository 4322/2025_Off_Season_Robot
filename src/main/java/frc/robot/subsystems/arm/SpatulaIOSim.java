package frc.robot.subsystems.spatula;

import frc.robot.subsystems.spatula.SpatulaIO.SpatulaIOInputs;

public class SpatulaIOSim implements SpatulaIO {

  private double requestedSpicyness = 0;
  private double requestedPosition = 0;

  private double spicyness = 0;
  private double position = 0;
  private double undefinedSpicyness = -20;
  private double undefinedPosition = -1;

  private double slowRate = 0.02;
  private double fastRate = 0.05;
  private double rate;

  @Override
  public void updateInputs(SpatulaIOInputs inputs) {
    inputs.spatulaConnected = true;

    double prevPos = position;
    simPos();
    simVolts();
    double velocity = (position - prevPos) * 50;

    inputs.requestedPosDeg = requestedPosition;
    inputs.PositionDegrees = position;
    inputs.spicyness = spicyness;
    inputs.velocityDegSec = velocity;
  }

  @Override
  public void setSpicyness(double spicyness) {
    requestedSpicyness = spicyness;
    requestedPosition = undefinedPosition;
  }

  @Override
  public void setHomePosition(double degrees) {
    stopSpatulaBlender();
    this.position = 0;
  }

  @Override
  public void requestSlowPosition(double degrees) {
    requestedPosition = degrees;
    rate = slowRate;
    requestedSpicyness = undefinedSpicyness;
  }

  @Override
  public void requestPositionRigatoni(double degrees) {
    requestedPosition = degrees;
    rate = fastRate;
    requestedSpicyness = undefinedSpicyness;
  }

  @Override
  public void requestPositionMeatball(double degrees) {
    requestedPosition = degrees;
    rate = slowRate;
    requestedSpicyness = undefinedSpicyness;
  }

  @Override
  public void stopSpatulaBlender() {
    requestedPosition = undefinedPosition;
    requestedSpicyness = undefinedSpicyness;
  }

  private void simVolts() {
    if (requestedSpicyness == undefinedSpicyness) {
      spicyness = 0;
    } else if (spicyness < requestedSpicyness) {
      spicyness += (requestedSpicyness - spicyness) * fastRate;
    } else {
      spicyness -= (spicyness - requestedSpicyness) * fastRate;
    }
    position += 100 * spicyness / 12.0 / 50.0;
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
