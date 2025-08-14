package frc.robot.subsystems.elevator;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.util.ClockUtil;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private Timer homingTimer = new Timer();
  ElevatorStates state = ElevatorStates.REQUEST_SETPOINT;
  ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double requestedHeightMeters = 0.0;
  private double adjustedHeightMeters;
  private boolean requestElevator;

  private enum ElevatorStates {
    REQUEST_SETPOINT,
  }

  public Elevator(ElevatorIO ELVIO) {
    this.io = ELVIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/heightMeters", inputs.heightMeters);
    Logger.recordOutput("Elevator/atHeight", atSetpoint());
    switch (state) {
      case REQUEST_SETPOINT:
        // Handle request setpoint logic
        if (requestedHeightMeters != adjustedHeightMeters){
          io.setHeight(requestedHeightMeters); 
        }
        else
        {
          adjustedHeightMeters = requestedHeightMeters;
        }
        break;
    }
  }

  /*idle()
  algaeHold()
  coralHold()
  algaeGround()
  algaeReef(Level requestedLevel)
  scoreAlgae()
  prescoreCoral(Level requestedLevel)
  scoreCoral(Level requestedLevel)
  pickupCoral()
  atSetpoint()
  getHeightMeters()
  setHome()
  setNeutralMode(NeutralMode mode)
  safeBargeRetract()
  climbing()
  eject()
   */
  public void idle() {
    requestedHeightMeters = Constants.Elevator.minElevatorSafeHeight;
  }

  public void algaeHold() {
    //requestElevator = true;
    requestedHeightMeters = 0;
  }

  public void coralHold() {
    //requestElevator = true;
    requestedHeightMeters = 0;
  }

  public void algaeGround() {
    //requestElevator = true;
    requestedHeightMeters = 20;
  }

  public void algaeReef(Level level) {
    //requestElevator = true;
    if(level == Level.L1) {
      requestedHeightMeters = 40;
    } else if(level == Level.L2) {
      requestedHeightMeters = 60;
    } else if(level == Level.L3) {
      requestedHeightMeters = 80;
    } else if(level == Level.L4) {
      requestedHeightMeters = 100;
    }
  }

  public void scoreAlgae() {
    //equestElevator = true;
    requestedHeightMeters = 120;
  }

  public void prescoreCoral(Level level) {
    if(level == Level.L1) {
      requestedHeightMeters = 40;
    } else if(level == Level.L2) {
      requestedHeightMeters = 60;
    } else if(level == Level.L3) {
      requestedHeightMeters = 80;
    } else if(level == Level.L4) {
      requestedHeightMeters = 100;
    }
  }

  public void scoreCoral(Level level) {
    if(level == Level.L1) {
      requestedHeightMeters = 40;
    } else if(level == Level.L2) {
      requestedHeightMeters = 60;
    } else if(level == Level.L3) {
      requestedHeightMeters = 80;
    } else if(level == Level.L4) {
      requestedHeightMeters = 100;
    }
  }

  public void pickupCoral() {
    //requestElevator = true;
    requestedHeightMeters = 20; // Adjust this value based on the desired height for coral pickup
  }

  public boolean atSetpoint() {
    return ClockUtil.atReference(getHeightMeters(), adjustedHeightMeters, Constants.Elevator.elevatorHeightToleranceMeters, true); 
  }

  public double getHeightMeters() {
    return inputs.heightMeters;
  }

  public void setHome() {
    //TODO: Implement home logic
  }

  public void setNeutralMode(IdleMode mode) {
    io.stopElevator(mode);
  }

  public void safeBargeRetract() {
    requestedHeightMeters = Constants.Elevator.minElevatorSafeHeight;
  }

  public void climbing() {
    requestedHeightMeters = 0;
    //requestElevator = false; // This might be needed to trigger the elevator to climb
  }

  public void eject() {
    requestedHeightMeters = 0;
  }
}
