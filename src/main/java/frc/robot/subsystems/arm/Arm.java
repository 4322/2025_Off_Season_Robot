package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  public ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private Superstructure superstructure;
  public double minSafeArmDegree;
  public double maxElevatorSafeMeters = Constants.Elevator.scoringL4CoralMeters;

  public double requestedSetpoint;
  public double prevSetpoint;
  public double newSetpoint = 0;

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/angleDegrees", inputs.armPositionDegrees);
    Logger.recordOutput("Arm/atSetpoint", atSetpoint());

    if (superstructure.isCoralHeld()) {
      minSafeArmDegree = Constants.Arm.minArmSafeWithCoralDeg;
    } else {
      minSafeArmDegree = Constants.Arm.minArmSafeDeg;
    }

    superstructure.getElevatorHeight();
    // Safety Logic
    // newsetpoint = prevsetpoint when it can move with elevator

    if (Constants.Elevator.minElevatorSafeHeightMeters <= superstructure.getElevatorHeight()
        && maxElevatorSafeMeters >= superstructure.getElevatorHeight()) {
      prevSetpoint = newSetpoint;
    }
    if (requestedSetpoint < minSafeArmDegree
        && superstructure.getElevatorHeight() < Constants.Elevator.minElevatorSafeHeightMeters) {
      requestedSetpoint = minSafeArmDegree; // Do we want driver to have to input setpoint again?

    } else if (getAngleDegrees() > minSafeArmDegree && requestedSetpoint < minSafeArmDegree) {
      requestedSetpoint = minSafeArmDegree;
      // Don't let it go below the safe angle
      // Or should we make it so it just won't move?
    } else if (maxElevatorSafeMeters > superstructure.getElevatorHeight()
        && getAngleDegrees() >= Constants.Arm.safeBargeRetractAngleDeg) {

      // Make it so you can move it only to get to safe positon

    }

    if (prevSetpoint != requestedSetpoint) {
      io.requestPosition(requestedSetpoint);
      prevSetpoint = requestedSetpoint;
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

  public void algaeReef() {
    requestedSetpoint = Constants.Arm.descoringAlgaeDeg; // TODO: angle for L2
  }

  public void scoreAlgae(/*Side scoringSide*/ ) {}

  public void prescoreCoral(Level coralLevel) {
    setcoralheight(coralLevel);
  }

  public void scoreCoral(Level coralLevel) {
    setcoralheight(coralLevel);
  }

  public boolean atSetpoint() {
    return ClockUtil.atReference(
        inputs.armPositionDegrees, requestedSetpoint, Constants.Arm.setpointToleranceDegrees, true);
  }

  public void safeBargeRetract() {}

  public void setNeutralMode(IdleMode idlemode) {}

  public void climbing() {
    requestedSetpoint = 20; // TODO:
  }

  public void eject() {
    requestedSetpoint = 20; // TODO:
  }

  public double getAngleDegrees() {
    return inputs.armPositionDegrees;
  }

  private void setcoralheight(Level coralLevel) {
    switch (coralLevel) {
      case L1:
        requestedSetpoint = Constants.Arm.scoringL1CoralDeg;
        break;
      case L2:
        requestedSetpoint = Constants.Arm.scoringL2CoralDeg;
        break;
      case L3:
        requestedSetpoint = Constants.Arm.scoringL3CoralDeg;
        break;
      case L4:
        requestedSetpoint = Constants.Arm.scoringL4CoralDeg;
        break;
    }
  }
}
