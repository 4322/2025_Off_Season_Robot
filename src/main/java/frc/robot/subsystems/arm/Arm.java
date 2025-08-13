package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.util.ClockUtil;

public class Arm extends SubsystemBase {
  private ArmIO io;
  public ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
  private Superstructure superstructure;

  public int setpoint; // Degrees, 0 is horizontal to front of robot

  public enum Safety {
    WAIT_FOR_ELEVATOR,
    MOVING_WITH_ELEVATOR,
    ARM_CANT_MOVE, 
  }

  Safety safety = Safety.ARM_CANT_MOVE;

  public Arm(ArmIO ArmIO) {
    this.io = ArmIO;
  }

  @Override
  public void periodic() {

    RobotContainer.superstructure.getElevatorHeight();

    switch (safety) {
      case ARM_CANT_MOVE:
        if (superstructure.getState() == Superstates.IDLE) {
          safety = Safety.WAIT_FOR_ELEVATOR;
        } else if (superstructure.getElevatorHeight()
            >= Constants.Elevator.minElevatorSafeHeightIdle) {
          safety = Safety.MOVING_WITH_ELEVATOR;
        }
        break;
      case WAIT_FOR_ELEVATOR:
        if (Constants.Elevator.minElevatorSafeHeightIdle
            <= superstructure.getElevatorHeight()) {
          safety = Safety.MOVING_WITH_ELEVATOR;
        }
        break;
      case MOVING_WITH_ELEVATOR:
        if (setpoint >= Constants.Arm.minArmSafeAngle
            && superstructure.getElevatorHeight()
                < Constants.Elevator.minElevatorSafeHeightIdle) {
          safety = Safety.WAIT_FOR_ELEVATOR;
        } else if (getAngleDegrees() > Constants.Arm.minArmSafeAngle && setpoint < Constants.Arm.minArmSafeAngle) {
          safety = Safety.WAIT_FOR_ELEVATOR;
        } else {
          io.setPosition(Rotation2d.fromDegrees(setpoint));
        }
        break;
    }
  }

  public void idle() {
    setpoint = 0;
  }

  public void algaeHold() {
    setpoint = 20; // TODO: angle for algae hold
  }

  public void coralHold() {
    setpoint = 20; // TODO:
  }

  public void algaeGround() {
    setpoint = 20; // TODO:
  }

  public void algaeReef(Level algaeLevel) {
    switch (algaeLevel) {
      case L1:
        setpoint = 30; // TODO: angle for L1
        break;
      case L2:
        setpoint = 60; // TODO: angle for L2
        break;
      case L3:
        setpoint = 90; // TODO: angle for L3
        break;
      case L4:
        setpoint = 120; // TODO: angle for L4
        break;
    }
  }

  public void scoreAlgae() {}

  public void prescoreCoral(Level coralLevel) {
    switch (coralLevel) {
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
    }
  }

  public void scoreCoral(Level coralLevel) {
    switch (coralLevel) {
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
    }
  }

  public boolean atSetpoint() {
    return ClockUtil.atReference(
        armInputs.armPositionRad, setpoint, Constants.Arm.setpointToleranceMeters, true);
  }

  public void safeBargeRetract() {}

  public void setHome() {
    setpoint = 0; // TODO:
  }

  public void setNeutralMode(IdleMode idlemode) {
    io.stopArmMotor(idlemode);
  }

  public void climbing() {
    setpoint = 20; // TODO:
  }

  public void eject() {
    setpoint = 20; // TODO:
  }

  public double getAngleDegrees() {
    return armInputs.armPositionRad;
  }
}
