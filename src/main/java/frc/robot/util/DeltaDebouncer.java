package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

// TODO add more comments; Change high to low etc.
public class DeltaDebouncer {
  public enum Mode {
    INSTANT, // Only trigger if a single change is greater than the threshold; Checks the current
    // value against the last value calculated
    CUMULATIVE // Trigger if smaller changes add up to be greater than the threshold
  }

  public enum ChangeType {
    INCREASE, // Only trigger on increases in value
    DECREASE, // Only trigger on decreases in value
    BOTH // Trigger on both increases and decreases in value
  }

  private final double deltaThreshold; // Minimum change in value to trigger the debounce logic
  private final double
      debounceTimeSeconds; // Amount of time to wait when the delta exceeds the threshold before
  // returning true
  private final double
      maxAccumulationSeconds; // Only used in cumulative mode; Amount of time to accumulate changes
  // before resetting the baseline value
  private final Mode mode;
  private final ChangeType changeType;

  private double lastValue;
  private double baselineValue;
  private boolean firstUpdate = true;
  private boolean correctChange;

  private Timer debounceTimer = new Timer();
  private Timer accumulationTimer = new Timer();

  public DeltaDebouncer(
      double debounceTimeSeconds,
      double deltaThreshold,
      Mode mode,
      double maxAccumulationSeconds,
      ChangeType changeType) {
    this.debounceTimeSeconds = debounceTimeSeconds;
    this.deltaThreshold = deltaThreshold;
    if (mode == Mode.CUMULATIVE) {
      this.maxAccumulationSeconds = maxAccumulationSeconds;
    } else {
      this.maxAccumulationSeconds = 0;
    }
    this.mode = mode;
    this.changeType = changeType;
    correctChange = false;
    debounceTimer.reset();
    accumulationTimer.reset();
  }

  public boolean calculate(double value) {
    // Handle the first update; Don't return true on it
    if (firstUpdate) {
      lastValue = value;
      baselineValue = value;
      firstUpdate = false;
      return false;
    }

    // Calculate the delta based on the mode
    double delta;
    if (mode == Mode.INSTANT) {
      delta = value - lastValue; // Use the last value for instant mode
    } else {
      delta = value - baselineValue; // Use the baseline value for cumulative mode
    }

    // Determine if the change matches the specified changeType
    correctChange = false;
    switch (changeType) {
      case INCREASE:
        correctChange = delta > deltaThreshold; // Trigger for positive deltas
        break;
      case DECREASE:
        correctChange = delta < -deltaThreshold; // Trigger for negative deltas
        break;
      case BOTH:
        correctChange =
            Math.abs(delta) >= deltaThreshold; // Trigger for both positive and negative deltas
        break;
    }

    // Checking if the correct change has occured and exceeded the threshold for long enough
    if (correctChange) {
      // If debounceTimer was stopped from the last execution, we reset it and start it again
      if (!debounceTimer.isRunning()) {
        debounceTimer.reset();
        debounceTimer.start();
      }

      // If the timer has elapsed the debounce time (due to the delta being constantly above the
      // threshold), we return true
      if (debounceTimer.hasElapsed(debounceTimeSeconds)) {
        lastValue = value;
        baselineValue = value;
        debounceTimer.stop();
        accumulationTimer.reset();
        accumulationTimer.start();
        firstUpdate = true;
        return true;
      }
    } else {
      // If the change is not correct, we reset the debounce timer
      if (mode == Mode.INSTANT) {
        debounceTimer.stop();
        debounceTimer.reset();
      } else {
        // Reset the baseline value if maxAccumulationSeconds have passed
        if (accumulationTimer.hasElapsed(maxAccumulationSeconds)) {
          baselineValue = value;
          accumulationTimer.reset();
          accumulationTimer.start();
        }
      }
    }

    // Update lastValue for use in the next caculation
    lastValue = value;
    return false;
  }
}
