package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.util.ClockUtil;

public class Arm extends SubsystemBase {
  private ArmIO io;
  public ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private Superstructure superstructure;
  public double minSafeArmDegree;
  public double maxElevatorSafeMeters = Constants.Elevator.scoringL4CoralMeters;
  private Constants.Arm armConstants;

  public double requestedSetpoint;
  public double prevSetpoint = -1000;
  public double newSetpoint;
  public double elevatorHeight = superstructure.getElevatorHeight();

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/atSetpoint", atSetpoint());

    if (superstructure.isCoralHeld()) {
      minSafeArmDegree = armConstants.minArmSafeWithCoralDeg;
    } else {
      minSafeArmDegree = armConstants.minArmSafeDeg;
    }

    // Safety Logic
    // Checks the logic checking for if it is in a dangerous position

    if (requestedSetpoint < minSafeArmDegree
        && elevatorHeight < Constants.Elevator.minElevatorSafeHeightMeters) {
      newSetpoint = minSafeArmDegree;
    } else if (maxElevatorSafeMeters > elevatorHeight
        && requestedSetpoint < armConstants.safeBargeRetractAngleDeg) {
      newSetpoint = prevSetpoint;

    } else {
      newSetpoint = requestedSetpoint; // Makes it to the requested setpoint if no dangers detected
    }

    // Moves the Elevator
    if (prevSetpoint != newSetpoint) {
      io.requestPosition(newSetpoint);
      prevSetpoint = newSetpoint;
    }
  }

  public void setManualInitialization() {
    io.setManualInitialization();
  }

  public void idle() {
    requestedSetpoint = armConstants.armIdleDeg;
  }

  public void algaeHold() {
    requestedSetpoint = armConstants.algaeHoldDeg;
  }

  public void coralHold() {
    requestedSetpoint = armConstants.coralHoldDeg;
  }

  public void algaeGround() {
    requestedSetpoint = armConstants.algaeGroundDeg;
  }

  public void algaeReef() {
    requestedSetpoint = armConstants.descoringAlgaeDeg;
  }

  public void scoreAlgae(/*Side scoringSide*/ ) {
    requestedSetpoint = armConstants.scoringAlgaeDeg;
  }

  public void prescoreCoral(Level coralLevel) {
    setcoralheight(coralLevel);
  }

  public void scoreCoral(Level coralLevel) {
    setcoralheight(coralLevel);
  }

  public boolean atSetpoint() {
    return ClockUtil.atReference(
        inputs.armPositionDegrees, requestedSetpoint, armConstants.setpointToleranceDegrees, true);
  }

  public void safeBargeRetract() {
    requestedSetpoint = armConstants.safeBargeRetractAngleDeg;
  }

  public void setNeutralMode(IdleMode mode) {
    prevSetpoint = -1000;
    io.stopArmMotor(mode);
  }

  public void climbing() {
    requestedSetpoint = armConstants.climbingDeg;
  }

  public void eject() {
    requestedSetpoint = armConstants.ejectDeg;
  }

  public void slowmode() {
    io.setSpeed(50, 50);
  }

  public double getAngleDegrees() {
    return inputs.armPositionDegrees;
  }

  private void setcoralheight(Level coralLevel) {
    switch (coralLevel) {
      case L1:
        requestedSetpoint = armConstants.scoringL1CoralDeg;
        break;
      case L2:
        requestedSetpoint = armConstants.scoringL2CoralDeg;
        break;
      case L3:
        requestedSetpoint = armConstants.scoringL3CoralDeg;
        break;
      case L4:
        requestedSetpoint = armConstants.scoringL4CoralDeg;
        break;
    }
  }
}
