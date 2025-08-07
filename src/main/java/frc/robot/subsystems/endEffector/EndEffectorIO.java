package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

    @AutoLog
    public static class EndEffectorIOInputs {
        public double endEffectorMotorAppliedVoltage = 0.0;
        public double endEffectorMotorSupplyCurrentAmps = 0.0;
        public double endEffectorMotorStatorCurrentAmps = 0.0;
        public double endEffectorMotorTempCelcius = 0.0;
        public double endEffectorMotorSpeedRotationsPerSec = 0.0;

        public boolean holdingSensorTriggered = false;
        // TODO add something for color?
    }

    public default void updateInputs(EndEffectorIOInputs inputs) {}

    public default void setEndEffectorMotorVoltage(double voltage) {}

    public default void stopEndEffectorMotor() {}

    public default void enableBrakeMode(boolean enable) {}

}
