package frc.robot.subsystems.arm;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.util.ClockUtil;

public class Arm extends SubsystemBase {
  private ArmIO io;
  public ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
  private Superstructure superstructure;

  public int setpoint; // Degrees, 0 is horizontal to front of robot

  public enum Safety {
    WAIT_FOR_ELEVATOR,
    MOVING_WITH_ELEVATOR,
  }

  Safety safety = Safety.WAIT_FOR_ELEVATOR;

  public Arm(ArmIO ArmIO) {
    this.io = ArmIO;
  }

  @Override
  public void periodic() {

    RobotContainer.superstructure.getElevatorHeight();

    switch (safety) {
        /*Notes:
         - There are 2 min elevator heights
          > Idle
          > Floor Pick up
          Im guessing that there will be 2 if statements and it detects weither it was requested or not
          ___
          See if it would work with Coral held to Prescore coral
          > We will use the idle min height for this
          > move elevator to min setpoint then move arm to setpoint
          > If request move out from idle then move both at same time
          > So if in idle position we should be able to move both at same time no matter the angle
          _____
          See if it would work with Idle to intake algae floor
          > We will use the intake algae floor min height for this
          > move both up at same time and arm out
          _____
          Prescore coral to Eject immediately into algae intake floor
          > We will use the Floor Pick up min height for this
          > So prescore coral to eject will look like arm move a bit but what if it dosen't wait for the setpoint to be reached
          -Maybe add the safety so it won't eject if arm and elevator are not in the right position
          _____
          > Arm is based on the elevators height
          > Arm shouldn't move if elevator is below the min safe height
          _____
          In what cases should we wait for the elevator?
          > If elevator is below the min safe height then we should wait for it to be at the min safe height before moving the arm
          > If arm reaches the min safe position but elevator is below the safe height
          > Well if arm is in a safe position but it is commanded to go under the safe positon
          //TODO
          _____
          In what case should we allow it to move again?
          > Once elevator is at the min safe height then we can move the arm
          > If it is requested to go up from Idle
          //TODO: Figure out other cases
          ____
          //TODO: Figure out how to make tolerence to work with the logic aka where or how to put it

          //TODO: Figure out when to swtitch between the two min heights
          ____





        */
      case WAIT_FOR_ELEVATOR:
        if (superstructure.getState() == Superstates.IDLE) { //Maybe have a prev state?
          safety = Safety.MOVING_WITH_ELEVATOR;
        }
        else if (Constants.Elevator.minElevatorSafeHeightIdle
            <= superstructure.getElevatorHeight()) {
          safety = Safety.MOVING_WITH_ELEVATOR;
        }
        break;
      case MOVING_WITH_ELEVATOR:
        if (setpoint >= Constants.Arm.minArmSafeAngle
            && superstructure.getElevatorHeight()
                < Constants.Elevator.minElevatorSafeHeightIdle) {
          safety = Safety.WAIT_FOR_ELEVATOR;
        } else if (getAngleDegrees() > Constants.Arm.minArmSafeAngle && setpoint < Constants.Arm.minArmSafeAngle) {
          safety = Safety.WAIT_FOR_ELEVATOR;
        } else {
          io.setPosition(Rotation2d.fromDegrees(setpoint));
        }
        break;
    }
  }

  public void idle() {
    setpoint = 0;
  }

  public void algaeHold() {
    setpoint = 20; // TODO: angle for algae hold
  }

  public void coralHold() {
    setpoint = 20; // TODO:
  }

  public void algaeGround() {
    setpoint = 20; // TODO:
  }

  public void algaeReef(Level algaeLevel) {
    switch (algaeLevel) {
      case L1:
        setpoint = 30; // TODO: angle for L1
        break;
      case L2:
        setpoint = 60; // TODO: angle for L2
        break;
      case L3:
        setpoint = 90; // TODO: angle for L3
        break;
      case L4:
        setpoint = 120; // TODO: angle for L4
        break;
    }
  }

  public void scoreAlgae() {}

  public void prescoreCoral(Level coralLevel) {
    switch (coralLevel) {
      case L1:
        setpoint = 30; // Example angle for L1
        break;
      case L2:
        setpoint = 60; // Example angle for L2
        break;
      case L3:
        setpoint = 90; // Example angle for L3
        break;
      case L4:
        setpoint = 120; // Example angle for L4
        break;
    }
  }

  public void scoreCoral(Level coralLevel) {
    switch (coralLevel) {
      case L1:
        setpoint = 30; // Example angle for L1
        break;
      case L2:
        setpoint = 60; // Example angle for L2
        break;
      case L3:
        setpoint = 90; // Example angle for L3
        break;
      case L4:
        setpoint = 120; // Example angle for L4
        break;
    }
  }

  public boolean atSetpoint() {
    return ClockUtil.atReference(
        armInputs.armPositionRad, setpoint, Constants.Arm.setpointToleranceMeters, true);
  }

  public void safeBargeRetract() {}

  public void setHome() {
    setpoint = 0; // TODO:
  }

  public void setNeutralMode(IdleMode idlemode) {
    io.stopArmMotor(idlemode);
  }

  public void climbing() {
    setpoint = 20; // TODO:
  }

  public void eject() {
    setpoint = 20; // TODO:
  }

  public double getAngleDegrees() {
    return armInputs.armPositionRad;
  }
}
