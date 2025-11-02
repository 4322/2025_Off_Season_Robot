package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BabyTunerX;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.util.ClockUtil;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double minSafeArmDegree;
  private double minElevatorHeight;

  private double requestedSetpoint;
  private double prevSetpoint = -1000;
  private double newSetpoint;
  private double currentScoreSetpoint;
  private double elevatorHeight;
  private boolean isHomed;
  private boolean inSync = true;

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/atSetpoint", atSetpoint());
    Logger.recordOutput("Arm/TargetAngle", requestedSetpoint);
    if (DriverStation.isDisabled()) {

    } else {
      if (isHomed) {
        if (Math.abs(
                inputs.PositionDegrees
                    + Constants.Arm.OffsetEncoderDeg
                    - inputs.encoderArmRotations % 1.0 * 360)
            <= Constants.Arm.syncToleranceDegrees) {
          inSync = true;
        } else if (!isMoving()) {
          inSync = false;
        }
        Logger.recordOutput("Arm/inSync", inSync);
        switch (Constants.armMode) {
          case OPEN_LOOP:
            double x = -RobotContainer.driver.getRightX();
            io.setVoltage(x * x * x * 12.0);
            break;
          case TUNING:
            Double newPos =
                BabyTunerX.run(0, io.getTalonFX(), "Arm", inputs.PositionDegrees, "degrees");
            if (newPos != null) {
              io.requestPositionCoral(newPos);
            }
            break;
          case DISABLED:
            break;
          case NORMAL:
            if (RobotContainer.getSuperstructure().isCoralHeld()) {
              minSafeArmDegree = Constants.Arm.minArmSafeWithCoralDeg;
              minElevatorHeight = Constants.Elevator.minElevatorSafeWithCoralMeters;
            } else {
              minSafeArmDegree = Constants.Arm.minArmSafeDeg;
              minElevatorHeight = Constants.Elevator.minElevatorSafeHeightMeters;
            }

            // Safety Logic
            // Checks the logic checking for if it is in a dangerous position

            elevatorHeight = RobotContainer.getSuperstructure().getElevatorHeight();

            if (requestedSetpoint < minSafeArmDegree
                && elevatorHeight < (minElevatorHeight - Constants.Elevator.bufferHeightMeters)
                && getAngleDegrees()
                    > (minSafeArmDegree
                        - Constants.Arm
                            .bufferDeg)) { // So if the requested setpoint is under the min
              // safe angle and the elevator is too low the arm
              // will go to min safe angle
              newSetpoint = minSafeArmDegree;
            } else if (requestedSetpoint > Constants.Arm.armIdleDeg && elevatorHeight < (minElevatorHeight - Constants.Elevator.bufferHeightMeters)){
              newSetpoint = Constants.Arm.armIdleDeg; 
            } //So we dont move arm when in cradle
            else {
              newSetpoint =
                  requestedSetpoint; // Makes it to the requested setpoint if no dangers detected
            }

            if (prevSetpoint != newSetpoint || Constants.continuousNitrateRequestsEnabled) {
              if (RobotContainer.getSuperstructure().isAlgaeHeld()) {
                io.requestPositionAlgae(newSetpoint);
              } else {
                io.requestPositionCoral(newSetpoint);
              }
              prevSetpoint = newSetpoint;
            }
        }
      }
    }
  }

  public void setHomePosition() {
    inputs.PositionDegrees = 0;
    io.setHomePosition(Units.degreesToRotations(Constants.Arm.OffsetEncoderDeg));
    isHomed = true;
    idle(); // must have a valid initial position request when enabled
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

  public void scoreAlgae(boolean scoreBackSide) {
    if (scoreBackSide) {
      requestedSetpoint = Constants.Arm.scoringBacksideAlgaeDeg;
    } else {
      requestedSetpoint = Constants.Arm.scoringAlgaeDeg;
    }
  }

  // Reset the setpoint so we can send a new request
  public void reset() {
    prevSetpoint = -1;
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
    switch (level) {
      case L1:
        requestedSetpoint = Constants.Arm.scoringL1CoralDeg;
        currentScoreSetpoint = Constants.Arm.scoringL1CoralDeg;
        break;
      case L2:
        requestedSetpoint = Constants.Arm.scoringL2CoralDeg;
        currentScoreSetpoint = Constants.Arm.scoringL2CoralDeg;
        break;
      case L3:
        requestedSetpoint = Constants.Arm.scoringL3CoralDeg;
        currentScoreSetpoint = Constants.Arm.scoringL3CoralDeg;
        break;
      case L4:
        requestedSetpoint = Constants.Arm.scoringL4CoralDeg;
        currentScoreSetpoint = Constants.Arm.scoringL4CoralDeg;
        break;
    }
  }

  public double scoreReleaseSetpoint() {
    return (currentScoreSetpoint + 4);
  }

  public boolean atSetpoint() {
    if (requestedSetpoint == Constants.Arm.scoringBacksideAlgaeDeg
        || requestedSetpoint == Constants.Arm.scoringAlgaeDeg) {
      return true;
    } else {
      return ClockUtil.atReference(
          inputs.PositionDegrees, requestedSetpoint, Constants.Arm.setpointToleranceDegrees, true);
    }
  }

  public void safeBargeRetract() {
    requestedSetpoint = Constants.Arm.safeBargeRetractDeg;
  }

  public void stop() {
    io.stopArmMotor();
    reset();
  }

  public double emergencyHoming() {
    isHomed = false;
    io.setVoltage(Constants.Arm.intializationVoltage);
    return inputs.velocityDegSec;
  }

  public void setEmergencyHomingComplete() {
    inputs.PositionDegrees = Constants.Arm.hittingIndexerDegrees;
    io.setHomePosition(
        Units.degreesToRotations(Constants.Arm.OffsetEncoderDeg + inputs.PositionDegrees));
    setReHome();
  }

  public void setReHome() {
    isHomed = true;
    idle();
  }

  public boolean isMoving() {
    return !atSetpoint() || inputs.velocityDegSec > Constants.Arm.initializationCompleteSpeed;
  }

  public void climbing() {
    requestedSetpoint = Constants.Arm.climbingDeg;
  }

  public void eject() {
    requestedSetpoint = Constants.Arm.ejectDeg;
  }

  public double getAngleDegrees() {
    return inputs.PositionDegrees;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }
}
