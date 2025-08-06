package frc.robot.subsystems.endEffector;


import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {

  public enum EndEffectorStates {}

  public EndEffector() {}

  @Override
  public void periodic() {}

  public void idle() {}

  public void intakeAlgae() {}

  public void intakeCoral() {}


  public void releaseAlgae() {}

  public boolean hasAlgae() {
    return true; // temp
  }

  public boolean hasCoral() {
    return true; // temp
  }

  public void setNeutralMode(IdleMode mode) {}
}
