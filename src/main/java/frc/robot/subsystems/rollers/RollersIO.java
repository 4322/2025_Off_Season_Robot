package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {

    @AutoLog
    public static class RollersIOInputs {
        public double feederAppliedVoltage = 0.0;
        public double feederSupplyCurrentAmps = 0.0;
        public double feederStatorCurrentAmps = 0.0;
        public double feederTempCelcius = 0.0;
        public double feederSpeedRotationsPerSec = 0.0;
    }

    public default void updateInputs(RollersIOInputs inputs) {}

    public default void setFeederVoltage(double voltage) {}

    public default void stopFeeder() {}

}