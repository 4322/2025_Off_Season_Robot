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
  private Timer initializationTimer = new Timer();
  ElevatorStates state = ElevatorStates.INITIALIZATIONPROCEDURE;
  ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double requestedHeightMeters = 0.0;

  private enum ElevatorStates {
    INITIALIZATIONPROCEDURE,
    WAIT_FOR_ARM,
    REQUEST_SETPOINT,
  }

  public Elevator(ElevatorIO ELVIO) {
    this.io = ELVIO;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/atHeight", atHeight());
    Logger.recordOutput("Elevator/ElevatorStates", state.toString());
    switch (state) {
      case INITIALIZATIONPROCEDURE:
        initializationTimer.start();
        io.setVoltage(Constants.Elevator.intializationVoltage);
        //setup initialization procedure logic
        if(initializationTimer.hasElapsed(Constants.Elevator.initializationTimerThresholdSecs) 
        && Math.abs(inputs.velocityMetersSecond) 
        < Constants.Elevator.initializationVelocityMetersThresholdPerSecs){ 
          io.setVoltage(0); //idk value
          io.setElevatorEncoder();
          initializationTimer.stop();
          initializationTimer.reset();
          state = ElevatorStates.WAIT_FOR_ARM;
        }
      case WAIT_FOR_ARM:
        if (RobotContainer.superstructure.getArmAngle() >= Constants.Arm.minArmSafeAngle) {
          state = ElevatorStates.REQUEST_SETPOINT;
        }
        break;
      case REQUEST_SETPOINT:
        io.requestHeight(requestedHeightMeters); 
        break;
    }
  }


  public void idle() {
    requestedHeightMeters = Constants.Elevator.minElevatorSafeHeightMeters;
  }

  public void algaeHold() {
    //requestElevator = true;
    requestedHeightMeters = Constants.Elevator.algaeHoldMeters;
  }

  public void coralHold() {
    //requestElevator = true;
    requestedHeightMeters = Constants.Elevator.minElevatorSafeHeightMeters;
  }

  public void algaeGround() {
    //requestElevator = true;
    requestedHeightMeters = Constants.Elevator.algaeGroundHeightMeters;
  }

  public void algaeReef(Level level) {
    //requestElevator = true;
    switch (level) {
      case L1:
        requestedHeightMeters = Constants.Elevator.algaeReefL1HeightMeters;
        break;
      case L2:
        requestedHeightMeters = Constants.Elevator.algaeReefL2HeightMeters;
        break;
      case L3:
        requestedHeightMeters = Constants.Elevator.algaeReefL3HeightMeters;
        break;
    }
  }

  public void scoreAlgae() {
    //equestElevator = true;
    requestedHeightMeters = Constants.Scoring.maxElevatorSafeHeightMeters;
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
        requestedHeightMeters = Constants.Scoring.scoreCoralL1HeightMeters;
        break;
      case L2:
        requestedHeightMeters = Constants.Scoring.scoreCoralL2HeightMeters;
        break;
      case L3:
        requestedHeightMeters = Constants.Scoring.scoreCoralL3HeightMeters;
        break;
      case L4:
        requestedHeightMeters = Constants.Scoring.scoreCoralL4HeightMeters;
        break;
    }
  }

  public void pickupCoral() {
    //requestElevator = true;
    requestedHeightMeters = Constants.Elevator.pickupCoralHeightMeters; // Adjust this value based on the desired height for coral pickup
  }

  public boolean atHeight() {
    return ClockUtil.atReference(getHeightMeters(), requestedHeightMeters, Constants.Elevator.elevatorHeightToleranceMeters, true); 
  }

  public double getHeightMeters() {
    return inputs.heightMeters;
  }

  public void setHome() {
    io.setElevatorEncoder();
  }

  public void setNeutralMode(IdleMode mode) {
    io.setNeutralMode(mode);
  }

  public void safeBargeRetract() {
    if((getHeightMeters() >= Constants.Elevator.safeBargeRetractHeightMeters) && (RobotContainer.superstructure.getArmAngle() == Constants.Arm.maxArmSafeAngle)){
    requestedHeightMeters = Constants.Elevator.safeBargeRetractHeightMeters;
    }
  }

  public void climbing() {
    //needs work
  }

  public void eject() {
    if((getHeightMeters() >= Constants.Elevator.ejectSafeHeightMeters) && (RobotContainer.superstructure.getArmAngle() == Constants.Arm.maxArmSafeAngle)){ 
    requestedHeightMeters = Constants.Elevator.ejectSafeHeightMeters;
    }
  }
  public void peformInitialization(){
    state = ElevatorStates.INITIALIZATIONPROCEDURE;
  }
}
