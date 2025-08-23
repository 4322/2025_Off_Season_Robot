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
  private Constants.Arm armConstants;

  public double requestedSetpoint;
  public double prevSetpoint;
  public double newSetpoint;

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
      minSafeArmDegree = armConstants.minArmSafeWithCoralDeg;
    } else {
      minSafeArmDegree = armConstants.minArmSafeDeg;
    }

    superstructure.getElevatorHeight();
    // Safety Logic
    // newsetpoint = prevsetpoint when it can move with elevator

    if (Constants.Elevator.minElevatorSafeHeightMeters <= superstructure.getElevatorHeight()
        && maxElevatorSafeMeters >= superstructure.getElevatorHeight()) {
      prevSetpoint = newSetpoint;
    }
      prevSetpoint = newSetpoint;
    }

    if (requestedSetpoint < minSafeArmDegree
        && superstructure.getElevatorHeight() < Constants.Elevator.minElevatorSafeHeightMeters) {
      requestedSetpoint = minSafeArmDegree; // Do we want driver to have to input setpoint again?

    } else if (getAngleDegrees() > minSafeArmDegree && requestedSetpoint < minSafeArmDegree) {
      requestedSetpoint = minSafeArmDegree;

    } else if (maxElevatorSafeMeters > superstructure.getElevatorHeight()) {
      requestedSetpoint = armConstants.safeBargeRetractAngleDeg;

    } else if (maxElevatorSafeMeters > superstructure.getElevatorHeight()
        && !superstructure
            .isCoralHeld()) // Don't want it to go to the safe retract when scoring as it will be
    // over max safe height before it scores
    {
      requestedSetpoint = armConstants.safeBargeRetractAngleDeg;
    }

    if (prevSetpoint != requestedSetpoint) {
      io.requestPosition(requestedSetpoint);
      prevSetpoint = requestedSetpoint;
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

  public void algaeReef() /*Noticed that the angle is the same for both heights*/ {
    requestedSetpoint = armConstants.descoringAlgaeDeg;
  }

  public void scoreAlgae(/*Side scoringSide. Why would scoring side matter?*/ ) {
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
    io.stopArmMotor(mode);
  }

  public void climbing() {
    requestedSetpoint = armConstants.climbingDeg;
    requestedSetpoint = armConstants.climbingDeg;
  }

  public void eject() {
    requestedSetpoint = armConstants.ejectDeg;
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
