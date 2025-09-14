package frc.robot.subsystems.elevator;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private Timer initializationTimer = new Timer();
  ElevatorStates state = ElevatorStates.UNHOMED;
  ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double requestedHeightMeters = 0.0;
  private double newHeightMeters = 0.0;

  private enum ElevatorStates {
    UNHOMED,
    INITIALIZATIONPROCEDURE,
    ELEVATOR_MOVEMENT
  }

  public Elevator(ElevatorIO ELVIO) {
    this.io = ELVIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/atHeight", atSetpoint());
    Logger.recordOutput("Elevator/ElevatorStates", state.toString());
    switch (state) {
      case UNHOMED:
        break;
      case INITIALIZATIONPROCEDURE:
      /* 
        initializationTimer.start();
        io.setVoltage(Constants.Elevator.intializationVoltage);
        // setup initialization procedure logic
        if (initializationTimer.hasElapsed(Constants.Elevator.initializationTimerThresholdSecs)
            && Math.abs(inputs.leaderMotorVelocityMetersSecond)
                < Constants.Elevator.initializationVelocityMetersThresholdPerSecs) {
          io.setVoltage(0.1); // idk value
          io.setPosition(Constants.Elevator.maxElevatorHeightMeters);
          initializationTimer.stop();
          initializationTimer.reset();
          state = ElevatorStates.ELEVATOR_MOVEMENT;
        }*/
      case ELEVATOR_MOVEMENT:
        if ((getElevatorHeightMeters() >= Constants.Elevator.ejectSafeHeightMeters)
            && (RobotContainer.getSuperstructure().getArmAngle() == Constants.Arm.ejectDeg)) {
          newHeightMeters = getElevatorHeightMeters();
        } else if ((getElevatorHeightMeters() >= Constants.Elevator.safeBargeRetractHeightMeters)
            && (RobotContainer.getSuperstructure().getArmAngle()
                == Constants.Arm.safeBargeRetractDeg)) {
          newHeightMeters = getElevatorHeightMeters();
        } else if (((RobotContainer.getSuperstructure().getArmAngle()
                <= Constants.Arm.minArmSafeDeg)
            && (getElevatorHeightMeters() == Constants.Elevator.minElevatorSafeHeightMeters))) {
          newHeightMeters = getElevatorHeightMeters();
        } else if ((RobotContainer.getSuperstructure().getArmAngle() < Constants.Arm.minArmSafeDeg)
            && (getElevatorHeightMeters() < Constants.Elevator.minElevatorSafeHeightMeters)) {
          newHeightMeters = Constants.Elevator.minElevatorSafeHeightMeters;
        } else {
          newHeightMeters = requestedHeightMeters;
        }
        if ((RobotContainer.getSuperstructure().getState() == Superstates.SCORE_CORAL)
            || (RobotContainer.getSuperstructure().getState() == Superstates.ALGAE_SCORE)) {
          io.requestSlowHeightMeters(newHeightMeters);
        } else {
          io.requestHeightMeters(newHeightMeters);
        }
        break;
    }
  }

  public void idle() {
    requestedHeightMeters = Constants.Elevator.minElevatorSafeHeightMeters;
  }

  public void algaeHold() {
    // requestElevator = true;
    requestedHeightMeters = Constants.Elevator.algaeHoldMeters;
  }

  public void coralHold() {
    // requestElevator = true;
    requestedHeightMeters = Constants.Elevator.minElevatorSafeHeightMeters;
  }

  public void algaeGround() {
    // requestElevator = true;
    requestedHeightMeters = Constants.Elevator.algaeGroundHeightMeters;
  }

  public void algaeReef(Level level) {
    // requestElevator = true;
    // requestElevator = true;
    switch (level) {
      case L2:
        requestedHeightMeters = Constants.Elevator.algaeReefL2HeightMeters;
        break;
      case L3:
        requestedHeightMeters = Constants.Elevator.algaeReefL3HeightMeters;
        break;
    }
  }

  public void scoreAlgae() {
    // equestElevator = true;
    // equestElevator = true;
    requestedHeightMeters = Constants.Elevator.scoreAlgaeHeightMeters;
  }

  public void prescoreCoral(Level level) {
    switch (level) {
      case L1:
        requestedHeightMeters = Constants.Elevator.prescoreCoralL1HeightMeters;
        break;
      case L2:
        requestedHeightMeters = Constants.Elevator.prescoreCoralL2HeightMeters;
        break;
      case L3:
        requestedHeightMeters = Constants.Elevator.prescoreCoralL3HeightMeters;
        break;
      case L4:
        requestedHeightMeters = Constants.Elevator.prescoreCoralL4HeightMeters;
        break;
    }
  }

  public void scoreCoral(Level level) {
    switch (level) {
      case L1:
        requestedHeightMeters = Constants.Elevator.scoreCoralL1HeightMeters;
        break;
      case L2:
        requestedHeightMeters = Constants.Elevator.scoreCoralL2HeightMeters;
        break;
      case L3:
        requestedHeightMeters = Constants.Elevator.scoreCoralL3HeightMeters;
        break;
      case L4:
        requestedHeightMeters = Constants.Elevator.scoreCoralL4HeightMeters;
        break;
    }
  }

  public void pickupCoral() {
    // requestElevator = true;
    requestedHeightMeters =
        Constants.Elevator
            .pickupCoralHeightMeters; // Adjust this value based on the desired height for coral
    // pickup
  }

  public boolean atSetpoint() {
    return ClockUtil.atReference(
        getElevatorHeightMeters(),
        requestedHeightMeters,
        Constants.Elevator.elevatorHeightToleranceMeters,
        true);
  }

  public double getElevatorHeightMeters() {
    return inputs.leaderMotorheightMeters;
  }

  public void setHomePosition() {
    io.setPosition(Constants.Elevator.homeHeightMeters);
    state = ElevatorStates.ELEVATOR_MOVEMENT;
  }

  public void setNeutralMode(IdleMode idleMode) {
    io.setNeutralMode(idleMode);
    idle();
  }

  public void safeBargeRetract() {
    requestedHeightMeters = Constants.Elevator.safeBargeRetractHeightMeters;
  }

  public void climbing() {
    // needs work
  }

  public void eject() {
    requestedHeightMeters = Constants.Elevator.ejectSafeHeightMeters;
  }

  public void peformInitialization() {
    state = ElevatorStates.INITIALIZATIONPROCEDURE;
  }
}
