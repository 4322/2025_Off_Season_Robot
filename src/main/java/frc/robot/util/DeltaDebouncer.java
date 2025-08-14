
package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class DeltaDebouncer {
    public enum Mode {
        INSTANT, // Only trigger if a single change is greater than the threshold
        CUMULATIVE // Trigger if smaller changes add up to be greater than the threshold
    }

    private final double deltaThreshold;
    private final double debounceTimeSeconds;
    private final double maxAccumulationSeconds;
    private final Mode mode;

    private double lastValue;
    private double baselineValue;
    private boolean firstUpdate = true;

    private Timer debounceTimer = new Timer();
    private Timer accumulationTimer = new Timer();

    DeltaDebouncer(double debounceTimeSeconds, double deltaThreshold, Mode mode, double maxAccumulationSeconds) {
        this.debounceTimeSeconds = debounceTimeSeconds;
        this.deltaThreshold = deltaThreshold;
        if (mode == Mode.CUMULATIVE) {
            this.maxAccumulationSeconds = maxAccumulationSeconds;
        } else {
            this.maxAccumulationSeconds = 0;
        }
        this.mode = mode;
        debounceTimer.reset();
        debounceTimer.start();
        accumulationTimer.reset();
        accumulationTimer.start();
    }

    public boolean calculate(double value) {
        // Don't trigger on first update
        if (firstUpdate) {
            lastValue = value;
            baselineValue = value;
            firstUpdate = false;
            return false;
        }

        double delta;
        if (mode == Mode.INSTANT) {
            delta = Math.abs(value - lastValue);
        } else {
            delta = Math.abs(value - baselineValue);
        }

        // Checking if delta has exceeded the threshold for long enough
        if (delta >= deltaThreshold) {
            if (!debounceTimer.isRunning()) {
                debounceTimer.reset();
                debounceTimer.start();
            }

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
            if (mode == Mode.INSTANT) {
                debounceTimer.stop(); // Reset if the delta is below the threshold
                debounceTimer.reset();
            } else {
                // Reset accumulation if maxAccumulationSeconds have passed
                if (accumulationTimer.hasElapsed(maxAccumulationSeconds)) {
                    baselineValue = value;
                    accumulationTimer.reset();
                    accumulationTimer.start();
                }
            }
            
        }

        lastValue = value;
        return false;
    }
}