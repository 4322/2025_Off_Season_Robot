package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private ArmIO io;

  public Arm(ArmIO ArmIO) {
    this.io = ArmIO;
  }

  public void idle() {
    // Set the arm to idle state
    // This could mean stopping the motors or setting a default position

  }

  public void algaeHold() {}

  public void coralHold() {}

  public void algaeGround() {}

  public void algaeReef(/*requestedLevel*/ ) {}

  public void scoreAlgae(/*ArmState scoringSide*/ ) {}

  public void prescoreCoral(/*ArmState requestedLevel*/ ) {}

  public void scoreCoral(/*ArmState requestedLevel*/ ) {}

  public boolean atSetpoint() {
    // TODO: Implement logic to check if arm is at setpoint
    return true;
  }

  public void safeBargeRetract() {}

  public void setHome() {}

  public void setNeutralMode(/*ArmState mode*/ ) {}

  public void climbing() {}

  public void eject() {}

  public void getAngleDegrees() {}
}
