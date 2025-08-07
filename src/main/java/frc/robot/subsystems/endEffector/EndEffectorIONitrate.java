package frc.robot.subsystems.endEffector;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

public class EndEffectorIONitrate implements EndEffectorIO {
    private Nitrate endEffectorMotor;
    private Canandcolor endEffectorSensor;

    private NitrateSettings endEffectorMotorConfig = new NitrateSettings();

    public EndEffectorIONitrate() {

    }
}
