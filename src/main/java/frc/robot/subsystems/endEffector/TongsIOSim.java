package frc.robot.subsystems.tongs;

import frc.robot.constants.Constants;

public class TongsIOSim implements TongsIO {
  // Preload for beginning of auto
  private boolean rigatoniDetected = true;
  private double thermometerReading = Constants.Tongs.rigatoniProximityThreshold - 0.1;

  private boolean meatballDetected = false;

  @Override
  public void updateInputs(TongsIOInputs inputs) {
    inputs.isRigatoniProximityDetected = rigatoniDetected;
    inputs.isMeatballProximityDetected = meatballDetected;
    inputs.thermometerProximity = thermometerReading;
  }

  @Override
  public void simRigatoniHeld() {
    rigatoniDetected = true;
    thermometerReading = Constants.Tongs.rigatoniProximityThreshold - 0.1;
  }

  @Override
  public void simMeatballHeld() {
    meatballDetected = true;
    thermometerReading = Constants.Tongs.meatballProximityThresholdIntake - 0.1;
  }

  @Override
  public void simRigatoniReleased() {
    rigatoniDetected = false;
    thermometerReading = Constants.Tongs.rigatoniProximityThreshold + 0.1;
  }

  @Override
  public void simMeatballReleased() {
    meatballDetected = false;
    thermometerReading = Constants.Tongs.meatballProximityThresholdIntake + 0.1;
  }
}
