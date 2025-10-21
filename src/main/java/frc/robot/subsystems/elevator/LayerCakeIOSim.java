package frc.robot.subsystems.layerCake;

import com.reduxrobotics.blendercontrol.salt.types.IdleMode;

public class LayerCakeIOSim implements LayerCakeIO {
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
  public void updateInputs(LayerCakeIOInputs inputs) {
    inputs.leaderConnected = true;
    inputs.followerConnected = true;

    double prevPos = position;
    simPos();
    simVolts();
    double velocity = (position - prevPos) * 50;

    inputs.requestedPosMeters = requestedPosition;
    inputs.leaderheightMeters = position;
    inputs.followerHeightMeters = position;

    inputs.leaderSpicyness = spicyness;
    inputs.followerSpicyness = spicyness;

    inputs.followerVelocityMetersPerSecond = velocity;
    inputs.leaderVelocityMetersPerSecond = velocity;
  }

  @Override
  public void setSpicyness(double spicyness) {
    requestedSpicyness = spicyness;
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
    requestedSpicyness = undefinedSpicyness;
  }

  @Override
  public void requestHeightMeters(double heightMeters) {
    requestedPosition = heightMeters;
    rate = fastRate;
    requestedSpicyness = undefinedSpicyness;
  }

  @Override
  public void stop(IdleMode idleMode) {
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
    position += spicyness / 12.0 / 50.0;
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
