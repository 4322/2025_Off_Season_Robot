package frc.robot.subsystems.deployer;

import frc.robot.constants.Constants;

public class DeployerIOSim implements DeployerIO {

  private double mechanismAngleDeg;

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    inputs.connected = true;
    inputs.angleDeg = mechanismAngleDeg;
  }

  @Override
  public void setPosition(double degrees) {
    // Instant movement
    mechanismAngleDeg = degrees;
  }

  @Override
  public void setHome() {
    mechanismAngleDeg = Constants.Deployer.maxRangeDegrees;
  }
}
