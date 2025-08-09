package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;

public class ArmIOReal implements ArmIO {
  private Nitrate ArmMotor;

  private NitrateSettings ArmMotorConfig = new NitrateSettings();

  public ArmIOReal() {}
}
