package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BabyAlchemist;
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
    if (DriverStation.isDisabled()) {

    } else {
      if (isHomed) {
        switch (Constants.armMode) {
          case OPEN_LOOP:
            double x = -RobotContainer.driver.getLeftX();
            io.setVoltage(x * x * x * 12.0);
            break;
          case TUNING:
            Double newPos =
                BabyAlchemist.run(0, io.getNitrate(), "Arm", inputs.PositionDegrees, "degrees");
            if (newPos != null) {
              io.requestPosition(newPos);
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
    }
  }

  public void setHomePosition() {
    io.setHomePosition();
    isHomed = true;
  }

  public void idle() {
    isSlow = false;
    requestedSetpoint = Constants.Arm.armIdleDeg;
  }

  public void algaeHold() {
    isSlow = false;
    requestedSetpoint = Constants.Arm.algaeHoldDeg;
  }

  public void coralHold() {
    isSlow = false;
    requestedSetpoint = Constants.Arm.coralHoldDeg;
  }

  public void algaeGround() {
    isSlow = false;
    requestedSetpoint = Constants.Arm.algaeGroundDeg;
  }

  public void algaeReef() {
    isSlow = false;
    requestedSetpoint = Constants.Arm.descoringAlgaeDeg;
  }

  public void scoreAlgae(/*Side scoringSide*/ ) {
    isSlow = false;
    requestedSetpoint = Constants.Arm.scoringAlgaeDeg;
  }

  public void reset() {
    isSlow = false;
    prevSetpoint = -1;
  }

  public void prescoreCoral(Level level) {
    isSlow = false;
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
        inputs.PositionDegrees, requestedSetpoint, Constants.Arm.setpointToleranceDegrees, true);
  }

  public void safeBargeRetract() {
    isSlow = false;
    requestedSetpoint = Constants.Arm.safeBargeRetractDeg;
  }

  public void stop(IdleMode mode) {
    prevSetpoint = -1000; // To reset the setpoint so we can send a new request
    io.stopArmMotor(mode);
  }

  public void climbing() {
    isSlow = false;
    requestedSetpoint = Constants.Arm.climbingDeg;
  }

  public void eject() {
    isSlow = false;
    requestedSetpoint = Constants.Arm.ejectDeg;
  }

  public double getAngleDegrees() {
    return inputs.PositionDegrees;
  }
}
