package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;

public class ArmIONitrate implements ArmIO {
  private Nitrate ArmMotor;

  private NitrateSettings ArmMotorConfig = new NitrateSettings();

  public ArmIONitrate() {}
}
