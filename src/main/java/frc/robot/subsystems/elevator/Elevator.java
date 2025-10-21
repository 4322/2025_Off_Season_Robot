package frc.robot.subsystems.elevator;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BabyAlchemist;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double requestedHeightMeters = 0.0;
  private double prevHeightMeters = 0.0;
  private double newElevatorHeight;
  private double currentScoreHeight;
  private boolean isSlow = false;
  private boolean isHomed;
  Superstructure superstructure = RobotContainer.getSuperstructure();
  private double minSafeArmDegree = 0.0;
  private double minElevatorHeight = 0.0;
  private boolean inSync = true;

  public Elevator(ElevatorIO ELVIO) {
    this.io = ELVIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/atHeight", atSetpoint());
    Logger.recordOutput("Elevator/TargetHeight", requestedHeightMeters);

    if (isHomed) {
      if (Math.abs(inputs.leaderheightMeters - inputs.followerHeightMeters)
          <= Constants.Elevator.syncToleranceMeters) {
        inSync = true;
      } else if (!isMoving()) {
        inSync = false;
      }
      Logger.recordOutput("Elevator/inSync", inSync);
      switch (Constants.elevatorMode) {
        case OPEN_LOOP:
          io.setVoltage(-RobotContainer.driver.getRightY() * 12.0);
          break;
        case TUNING:
          Double newPos =
              BabyAlchemist.run(
                  0, io.getNitrate(), "Elevator", inputs.leaderheightMeters, "meters");
          if (newPos != null) {
            io.requestHeightMeters(newPos);
            requestedHeightMeters = newPos;
          }
          break;
        case DISABLED:
          break;
        case NORMAL:
          double armAngle = RobotContainer.getSuperstructure().getArmAngle();

          if (RobotContainer.getSuperstructure().isRigatoniHeld()) {
            minSafeArmDegree = Constants.Arm.minArmSafeWithRigatoniDeg;
            minElevatorHeight = Constants.Elevator.minElevatorSafeWithRigatoniMeters;
          } else {
            minSafeArmDegree = Constants.Arm.minArmSafeDeg;
            minElevatorHeight = Constants.Elevator.minElevatorSafeHeightMeters;
          }

          if (RobotContainer.getSuperstructure().getState() != Superstates.END_EFFECTOR_RIGATONI_PICKUP
              && requestedHeightMeters < minElevatorHeight
              && armAngle < (minSafeArmDegree - Constants.Arm.bufferDeg)) {
            newElevatorHeight = minElevatorHeight;
          } else {
            newElevatorHeight = requestedHeightMeters;
          }

          if (prevHeightMeters != newElevatorHeight || Constants.continuousNitrateRequestsEnabled) {
            if (isSlow) {
              io.requestSlowHeightMeters(newElevatorHeight);
            } else {
              io.requestHeightMeters(newElevatorHeight);
            }
            prevHeightMeters = newElevatorHeight;
          }
          break;
      }
    }
  }

  public void idle() {
    requestedHeightMeters = Constants.Elevator.minElevatorSafeHeightMeters;
    isSlow = false;
  }

  public void meatballHold() {
    requestedHeightMeters = Constants.Elevator.meatballHoldMeters;
    isSlow = false;
  }

  public void rigatoniHold() {
    requestedHeightMeters = Constants.Elevator.minElevatorSafeWithRigatoniMeters;
    isSlow = false;
  }

  public void meatballGround() {
    requestedHeightMeters = Constants.Elevator.meatballGroundHeightMeters;
    isSlow = false;
  }

  public void reset() {
    isSlow = false;
    prevHeightMeters = -1;
  }

  public void meatballReef(Level level) {
    switch (level) {
      case L2:
        requestedHeightMeters = Constants.Elevator.meatballReefL2HeightMeters;
        break;
      case L3:
        requestedHeightMeters = Constants.Elevator.meatballReefL3HeightMeters;
        break;
      default:
        System.out.println("Invalid level in Elevator.meatballReef()");
        System.exit(-1); // die so someone has to fix this
    }
    isSlow = false;
  }

  public void scoreMeatball() {
    requestedHeightMeters = Constants.Elevator.scoreMeatballHeightMeters;
    isSlow = false;
  }

  public void prescoreRigatoni(Level level) {
    switch (level) {
      case L1:
        requestedHeightMeters = Constants.Elevator.prescoreRigatoniL1HeightMeters;
        break;
      case L2:
        requestedHeightMeters = Constants.Elevator.prescoreRigatoniL2HeightMeters;
        break;
      case L3:
        requestedHeightMeters = Constants.Elevator.prescoreRigatoniL3HeightMeters;
        break;
      case L4:
        requestedHeightMeters = Constants.Elevator.prescoreRigatoniL4HeightMeters;
        break;
    }
    isSlow = false;
  }

  public void scoreRigatoni(Level level) {
    switch (level) {
      case L1:
        requestedHeightMeters = Constants.Elevator.scoreRigatoniL1HeightMeters;
        currentScoreHeight = Constants.Elevator.scoreRigatoniL1HeightMeters;
        break;
      case L2:
        requestedHeightMeters = Constants.Elevator.scoreRigatoniL2HeightMeters;
        currentScoreHeight = Constants.Elevator.scoreRigatoniL2HeightMeters;
        break;
      case L3:
        requestedHeightMeters = Constants.Elevator.scoreRigatoniL3HeightMeters;
        currentScoreHeight = Constants.Elevator.scoreRigatoniL3HeightMeters;
        break;
      case L4:
        requestedHeightMeters = Constants.Elevator.scoreRigatoniL4HeightMeters;
        currentScoreHeight = Constants.Elevator.scoreRigatoniL4HeightMeters;
        break;
    }
    isSlow = false;
  }

  public double releaseRigatoniSetpoint() {
    return currentScoreHeight + 0.001;
  }

  public void pickupRigatoni() {
    requestedHeightMeters =
        Constants.Elevator
            .pickupRigatoniHeightMeters; // Adjust this value based on the desired height for rigatoni
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
    return inputs.leaderheightMeters;
  }

  public void setHomePosition() {
    io.setPosition(Constants.Elevator.homeHeightMeters);
    isHomed = true;
    isSlow = false;
    idle(); // must have a valid initial position request when enabled
  }

  public double emergencyHoming() {
    isHomed = false;
    io.setVoltage(Constants.Elevator.intializationVoltage);
    requestedHeightMeters = 0;
    return inputs.leaderVelocityMetersPerSecond;
  }

  public void setEmergencyHomingComplete() {
    io.setPosition(Constants.Elevator.maxElevatorHeightMeters);
    setReHome();
  }

  public void setReHome() {
    isHomed = true;
    idle();
  }

  public void stop(IdleMode idleMode) {
    io.stop(idleMode);
    reset();
    isSlow = false;
  }

  public boolean isMoving() {
    return !atSetpoint()
        || inputs.leaderVelocityMetersPerSecond > Constants.Elevator.initializationCompleteSpeed;
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
