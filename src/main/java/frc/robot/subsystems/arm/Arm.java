package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

public class Arm extends SubsystemBase {
  private ArmIO io;
  public ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private Superstructure superstructure;
  public double minSafeArmDegree;

  public double requestedSetpoint; // Degrees, 0 is horizontal to front of robot
  public double prevSetpoint =
      MathUtil.clamp(
          requestedSetpoint, Constants.Arm.minArmSafeAngle, Constants.Arm.maxArmSafeAngle);

  public enum Safety {
    WAIT_FOR_ELEVATOR,
    MOVING_WITH_ELEVATOR,
  }

  Safety safety = Safety.MOVING_WITH_ELEVATOR;

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
      minSafeArmDegree = Constants.Arm.minArmSafeAngleWithCoral;
    } else {
      minSafeArmDegree = Constants.Arm.minArmSafeAngle;
    }

    superstructure.getElevatorHeight();

    switch (safety) {
      case WAIT_FOR_ELEVATOR:
        prevSetpoint = minSafeArmDegree;
        if (Constants.Elevator.minElevatorSafeHeightMeters <= superstructure.getElevatorHeight()) {
          safety = Safety.MOVING_WITH_ELEVATOR;
        }
        break;
      case MOVING_WITH_ELEVATOR:
        if (requestedSetpoint < minSafeArmDegree
            && superstructure.getElevatorHeight()
                < Constants.Elevator.minElevatorSafeHeightMeters) {
          safety = Safety.WAIT_FOR_ELEVATOR;
        } else if (getAngleDegrees() > minSafeArmDegree && requestedSetpoint < minSafeArmDegree) {
          safety = Safety.WAIT_FOR_ELEVATOR;
        }
        break;
    }
    if (prevSetpoint != requestedSetpoint && Safety.MOVING_WITH_ELEVATOR == safety) {
      io.requestPosition(Rotation2d.fromDegrees(requestedSetpoint));
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
    requestedSetpoint = Constants.Arm.descoringAngleDegAlgae; // TODO: angle for L2
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

  public void setManualInitialization() {
    // TODO: Figure out how to configurate an encoder
  }

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
        requestedSetpoint = Constants.Arm.scoringL1AngleDegCoral;
        break;
      case L2:
        requestedSetpoint = Constants.Arm.scoringL2AngleDegCoral;
        break;
      case L3:
        requestedSetpoint = Constants.Arm.scoringL3AngleDegCoral;
        break;
      case L4:
        requestedSetpoint = Constants.Arm.scoringL4AngleDegCoral;
        break;
    }
  }
  private void getZeroOffset(){
    
  }

}
