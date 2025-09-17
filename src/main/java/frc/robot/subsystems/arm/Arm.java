package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double minSafeArmDegree;

  private double requestedSetpoint;
  private double prevSetpoint = -1000;
  private double newSetpoint;
  private double elevatorHeight;
  private boolean isSlow = false;
  private boolean isHomed;

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/atSetpoint", atSetpoint());
    Logger.recordOutput("Arm/TargetAngle", requestedSetpoint);

    if (Constants.Arm.manualontrol) {
      io.setVoltage(RobotContainer.driver.getLeftX() * 12.0);
    } else if (isHomed) {
      if (RobotContainer.getSuperstructure().isCoralHeld()) {
        minSafeArmDegree = Constants.Arm.minArmSafeWithCoralDeg;
      } else {
        minSafeArmDegree = Constants.Arm.minArmSafeDeg;
      }

      // Safety Logic
      // Checks the logic checking for if it is in a dangerous position

      elevatorHeight = RobotContainer.getSuperstructure().getElevatorHeight();
      if (requestedSetpoint < minSafeArmDegree
          && elevatorHeight
              < (Constants.Elevator.minElevatorSafeHeightMeters
                  - Constants.Elevator.bufferHeightMeters)
          && getAngleDegrees()
              > (minSafeArmDegree
                  - Constants.Arm.bufferDeg)) { // So if the requested setpoint is under the min
        // safe angle and the elevator is too low the arm
        // will go to min safe angle
        newSetpoint = minSafeArmDegree;
      } else {
        newSetpoint =
            requestedSetpoint; // Makes it to the requested setpoint if no dangers detected
      }

      if (prevSetpoint != newSetpoint) {
        if (isSlow) {
          io.requestSlowPosition(newSetpoint);
          prevSetpoint = newSetpoint;
        } else {
          io.requestPosition(newSetpoint);
          prevSetpoint = newSetpoint;
        }
      }
    }
  }

  public void setHomePosition() {
    io.setHomePosition();
    isHomed = true;
  }

  public void idle() {
    requestedSetpoint = Constants.Arm.armIdleDeg;
  }

  public void algaeHold() {
    requestedSetpoint = Constants.Arm.algaeHoldDeg;
  }

  public void coralHold() {
    requestedSetpoint = Constants.Arm.coralHoldDeg;
  }

  public void algaeGround() {
    requestedSetpoint = Constants.Arm.algaeGroundDeg;
  }

  public void algaeReef() {
    requestedSetpoint = Constants.Arm.descoringAlgaeDeg;
  }

  public void scoreAlgae(/*Side scoringSide*/ ) {
    requestedSetpoint = Constants.Arm.scoringAlgaeDeg;
  }

  public void prescoreCoral(Level level) {
    switch (level) {
      case L1:
        requestedSetpoint = Constants.Arm.prescoringL1CoralDeg;
        break;
      case L2:
        requestedSetpoint = Constants.Arm.prescoringL2CoralDeg;
        break;
      case L3:
        requestedSetpoint = Constants.Arm.prescoringL3CoralDeg;
        break;
      case L4:
        requestedSetpoint = Constants.Arm.prescoringL4CoralDeg;
        break;
    }
  }

  public void scoreCoral(Level level) {
    isSlow = true;
    switch (level) {
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

  public boolean atSetpoint() {
    return ClockUtil.atReference(
        inputs.armPositionDegrees, requestedSetpoint, Constants.Arm.setpointToleranceDegrees, true);
  }

  public void safeBargeRetract() {
    requestedSetpoint = Constants.Arm.safeBargeRetractDeg;
  }

  public void stop(IdleMode mode) {
    prevSetpoint = -1000;
    io.stopArmMotor(mode);
  }

  public void climbing() {
    requestedSetpoint = Constants.Arm.climbingDeg;
  }

  public void eject() {
    requestedSetpoint = Constants.Arm.ejectDeg;
  }

  public double getAngleDegrees() {
    return inputs.armPositionDegrees;
  }
}
