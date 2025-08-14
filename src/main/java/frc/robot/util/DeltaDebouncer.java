
package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class DeltaDebouncer {
    private final double deltaThreshold;
    private final double debounceTimeSeconds;

    private double lastValue;
    private boolean firstUpdate = true;

    private Timer timer = new Timer();

    DeltaDebouncer(double debounceTimeSeconds, double deltaThreshold) {
        this.debounceTimeSeconds = debounceTimeSeconds;
        this.deltaThreshold = deltaThreshold;
        timer.reset();
        timer.start();
    }

    public boolean calculate(double value) {
        // Don't trigger on first update
        if (firstUpdate) {
            lastValue = value;
            firstUpdate = false;
            return false;
        }

        double delta = Math.abs(value - lastValue);

        // Checking if delta has exceeded the threshold for long enough
        if (delta >= deltaThreshold) {
            if (!timer.isRunning()) {
                timer.reset();
                timer.start();
            }

            if (timer.hasElapsed(debounceTimeSeconds)) {
                lastValue = value;
                timer.stop();
                return true;
            }
        } else {
            timer.stop(); // Reset if the delta is below the threshold
            timer.reset();
        }

        return false;
    }
}