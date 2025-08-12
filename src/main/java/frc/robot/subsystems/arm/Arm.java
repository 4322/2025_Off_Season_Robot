package frc.robot.subsystems.arm;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.Level;


public class Arm extends SubsystemBase {
  private ArmIO io;

  private static final double MIN_SAFE_ARM_ANGLE = 20.0;
  private static final double MAX_SAFE_ARM_ANGLE = 20.0;
  public int setpoint = 0; // Degrees, 0 is horizontal to front of robot
 

  public Arm(ArmIO ArmIO) {
    this.io = ArmIO;
  }

  public enum Safety {
   
  }

    @Override
    public void periodic() {
      io.setPosition(Rotation2d.fromDegrees(setpoint));
    }

  public void idle() {}

  public void algaeHold() {}

  public void coralHold() {}

  public void algaeGround() {}

  public void algaeReef(Level level ) {
    switch(level) {
      case L1:
        setpoint = 30; // Example angle for L1
        break;
      case L2:
        setpoint = 60; // Example angle for L2
        break;
      case L3:
        setpoint = 90; // Example angle for L3
        break;
      case L4:
        setpoint = 120; // Example angle for L4
        break;
      default:
        setpoint = 0; // Default to horizontal if no level is specified
    }
  }

  public void scoreAlgae(/*ArmState scoringSide*/ ) {}

  public void prescoreCoral(Level level ) {
    switch(level) {
      case L1:
        setpoint = 30; // Example angle for L1
        break;
      case L2:
        setpoint = 60; // Example angle for L2
        break;
      case L3:
        setpoint = 90; // Example angle for L3
        break;
      case L4:
        setpoint = 120; // Example angle for L4
        break;
      default:
        setpoint = 0; // Default to horizontal if no level is specified
    }
  }

  public void scoreCoral(Level level) {
    switch(level) {
      case L1:
        setpoint = 30; // Example angle for L1
        break;
      case L2:
        setpoint = 60; // Example angle for L2
        break;
      case L3:
        setpoint = 90; // Example angle for L3
        break;
      case L4:
        setpoint = 120; // Example angle for L4
        break;
      default:
        setpoint = 0; // Default to horizontal if no level is specified
    }
  }

  public boolean atSetpoint() {
    // TODO: Implement logic to check if arm is at setpoint
    return true;
  }

  public void safeBargeRetract() {}

  public void setHome() {

  }

  public void setNeutralMode(/*ArmState mode*/ ) {}

  public void climbing() {}

  public void eject() {}

  public void getAngleDegrees() {}
}
