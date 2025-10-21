package frc.robot.subsystems.endEffector;

import frc.robot.constants.Constants;

public class EndEffectorIOSim implements EndEffectorIO {
  // Preload for beginning of auto
  private boolean rigatoniDetected = true;
  private double sensorReading = Constants.EndEffector.rigatoniProximityThreshold - 0.1;

  private boolean meatballDetected = false;

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.isRigatoniProximityDetected = rigatoniDetected;
    inputs.isMeatballProximityDetected = meatballDetected;
    inputs.sensorProximity = sensorReading;
  }

  @Override
  public void simRigatoniHeld() {
    rigatoniDetected = true;
    sensorReading = Constants.EndEffector.rigatoniProximityThreshold - 0.1;
  }

  @Override
  public void simMeatballHeld() {
    meatballDetected = true;
    sensorReading = Constants.EndEffector.meatballProximityThresholdIntake - 0.1;
  }

  @Override
  public void simRigatoniReleased() {
    rigatoniDetected = false;
    sensorReading = Constants.EndEffector.rigatoniProximityThreshold + 0.1;
  }

  @Override
  public void simMeatballReleased() {
    meatballDetected = false;
    sensorReading = Constants.EndEffector.meatballProximityThresholdIntake + 0.1;
  }
}
