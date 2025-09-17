package frc.robot.subsystems.elevator;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  ElevatorStates state = ElevatorStates.UNHOMED;
  ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double requestedHeightMeters = 0.0;
  private double prevHeightMeters = 0.0;
  private double newElevatorHeight;
  private boolean isSlow = false;

  private enum ElevatorStates {
    UNHOMED,
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
    Logger.recordOutput("Elevator/TargetHeight", requestedHeightMeters);

    if (Constants.Elevator.manualControl) {
      io.setVoltage(-RobotContainer.driver.getLeftY() * 12.0);
    } else {
      switch (state) {
        case UNHOMED:
          break;
        case ELEVATOR_MOVEMENT:
          double armAngle = RobotContainer.getSuperstructure().getArmAngle();
          // TODO: Safety logic unsafe when going from coral held to score L2, FIX
          if (armAngle > Constants.Arm.bufferDeg
              && requestedHeightMeters < Constants.Elevator.minElevatorSafeHeightMeters
              && armAngle < (Constants.Arm.minArmSafeDeg - Constants.Arm.bufferDeg)) {
            newElevatorHeight = Constants.Elevator.minElevatorSafeHeightMeters;
          } else {
            newElevatorHeight = requestedHeightMeters;
          }
          if (prevHeightMeters != newElevatorHeight) {
            if (isSlow) {
              io.requestSlowHeightMeters(newElevatorHeight);
            } else {
              io.requestHeightMeters(newElevatorHeight);
            }
            prevHeightMeters = requestedHeightMeters;
          }
          break;
      }
    }
  }

  public void idle() {
    requestedHeightMeters = Constants.Elevator.minElevatorSafeHeightMeters;
    isSlow = false;
  }

  public void algaeHold() {
    requestedHeightMeters = Constants.Elevator.algaeHoldMeters;
    isSlow = false;
  }

  public void coralHold() {
    requestedHeightMeters = Constants.Elevator.minElevatorSafeWithCoralHeightMeters;
    isSlow = false;
  }

  public void algaeGround() {
    requestedHeightMeters = Constants.Elevator.algaeGroundHeightMeters;
    isSlow = false;
  }

  public void algaeReef(Level level) {
    switch (level) {
      case L2:
        requestedHeightMeters = Constants.Elevator.algaeReefL2HeightMeters;
        break;
      case L3:
        requestedHeightMeters = Constants.Elevator.algaeReefL3HeightMeters;
        break;
      default:
        System.out.println("Invalid level in Elevator.algaeReef()");
        System.exit(-1); // die so someone has to fix this
    }
    isSlow = false;
  }

  public void scoreAlgae() {
    requestedHeightMeters = Constants.Elevator.scoreAlgaeHeightMeters;
    isSlow = false;
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
    isSlow = false;
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
    isSlow = true;
  }

  public void pickupCoral() {
    requestedHeightMeters =
        Constants.Elevator
            .pickupCoralHeightMeters; // Adjust this value based on the desired height for coral
    isSlow = false;
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
    isSlow = false;
  }

  public void stop(IdleMode idleMode) {
    io.stop(idleMode);
    isSlow = false;
  }

  public void safeBargeRetract() {
    requestedHeightMeters = Constants.Elevator.safeBargeRetractHeightMeters;
    isSlow = false;
  }

  public void eject() {
    requestedHeightMeters = Constants.Elevator.ejectHeightMeters;
    isSlow = false;
  }
}
