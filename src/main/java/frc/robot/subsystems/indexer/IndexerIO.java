package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        public double indexerMotorAppliedVoltage = 0.0;
        public double indexerMotorCurrentAmps = 0.0;
        public double indexerMotorStatorCurrentAmps = 0.0;
        public double indexerMotorTempCelcius = 0.0;
        public double indexerMotorSpeedRotationsPerSec = 0.0;

        public boolean indexerSensorTriggered = false;
        public boolean pickupAreaSensorTriggered = false;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setIndexerMotorVoltage(double voltage) {}

    public default void stopFeeder() {}

}
