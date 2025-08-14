package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.util.ClockUtil;

public class Arm extends SubsystemBase {
  private ArmIO io;
  public ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
  private Superstructure superstructure;

  public double requestedSetpoint; // Degrees, 0 is horizontal to front of robot
  public double adjustedSetpoint = MathUtil.clamp(requestedSetpoint, Constants.Arm.minArmSafeAngle, Constants.Arm.maxArmSafeAngle);

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
      case WAIT_FOR_ELEVATOR:
        if (Constants.Elevator.minElevatorSafeHeight <= superstructure.getElevatorHeight()) {
          safety = Safety.MOVING_WITH_ELEVATOR;
        }
        break;
      case MOVING_WITH_ELEVATOR:
        if (requestedSetpoint < Constants.Arm.minArmSafeAngle
            && superstructure.getElevatorHeight() < Constants.Elevator.minElevatorSafeHeight) {
          safety = Safety.WAIT_FOR_ELEVATOR;
        } else if (getAngleDegrees() > Constants.Arm.minArmSafeAngle
            && requestedSetpoint < Constants.Arm.minArmSafeAngle) {
          safety = Safety.WAIT_FOR_ELEVATOR;
        } else {
          adjustedSetpoint = requestedSetpoint;
        }
        break;
    }
    if (adjustedSetpoint != requestedSetpoint) {
      io.setPosition(Rotation2d.fromDegrees(requestedSetpoint));
    }
  }

  public void idle() {
    requestedSetpoint = 0;
  }

  public void algaeHold() {
    requestedSetpoint = 20; // TODO: angle for algae hold
  }

  public void coralHold() {
    requestedSetpoint = 20; // TODO:
  }

  public void algaeGround() {
    requestedSetpoint = 20; // TODO:
  }

  public void algaeReef(Level algaeLevel) {
    switch (algaeLevel) {
      case L1:
        requestedSetpoint = 30; // TODO: angle for L1
        break;
      case L2:
        requestedSetpoint = 60; // TODO: angle for L2
        break;
      case L3:
        requestedSetpoint = 90; // TODO: angle for L3
        break;
      case L4:
        requestedSetpoint = 120; // TODO: angle for L4
        break;
    }
  }

  public void scoreAlgae() {}

  public void prescoreCoral(Level coralLevel) {
    switch (coralLevel) {
      case L1:
        requestedSetpoint = 30; // Example angle for L1
        break;
      case L2:
        requestedSetpoint = 60; // Example angle for L2
        break;
      case L3:
        requestedSetpoint = 90; // Example angle for L3
        break;
      case L4:
        requestedSetpoint = 120; // Example angle for L4
        break;
    }
  }

  public void scoreCoral(Level coralLevel) {
    switch (coralLevel) {
      case L1:
        requestedSetpoint = 30; // Example angle for L1
        break;
      case L2:
        requestedSetpoint = 60; // Example angle for L2
        break;
      case L3:
        requestedSetpoint = 90; // Example angle for L3
        break;
      case L4:
        requestedSetpoint = 120; // Example angle for L4
        break;
    }
  }

  public boolean atSetpoint() {
    return ClockUtil.atReference(
        armInputs.armPositionDegrees, requestedSetpoint, Constants.Arm.setpointToleranceDegrees, true);
  }

  public void safeBargeRetract() {}

  public void setHome() {
    requestedSetpoint = 0; // TODO:
  }

  public void setNeutralMode(IdleMode idlemode) {
    io.stopArmMotor(idlemode);
  }

  public void climbing() {
    requestedSetpoint = 20; // TODO:
  }

  public void eject() {
    requestedSetpoint = 20; // TODO:
  }

  public double getAngleDegrees() {
    return armInputs.armPositionDegrees;
  }
}
