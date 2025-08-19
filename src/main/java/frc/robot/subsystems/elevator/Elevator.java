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
  private double adjustedHeightMeters;//NEEDS WORK
  private boolean requestElevator;

  private enum ElevatorStates {
    INITIALIZATIONPROCEDURE,
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
    switch (state) {
      case INITIALIZATIONPROCEDURE:
      case REQUEST_SETPOINT:
        // Work on setpoint logic
        if (requestedHeightMeters != adjustedHeightMeters){
          io.requestHeight(requestedHeightMeters); 
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
  private void requestLevel(Level level, double requestedHeight){
    switch (level){
      case L1:
        requestedHeightMeters = requestedHeight;
        break;
      case L2:
        requestedHeightMeters = requestedHeight;
        break;
      case L3:
        requestedHeightMeters = requestedHeight;
        break;
      case L4:
        requestedHeightMeters = requestedHeight;
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
        requestLevel(level, Constants.Elevator.algaeReefL1HeightMeters);
        break;
      case L2:
        requestLevel(level, Constants.Elevator.algaeReefL2HeightMeters);
        break;
      case L3:
        requestLevel(level, Constants.Elevator.algaeReefL3HeightMeters);
        break;
      case L4:
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
        requestLevel(level, Constants.Elevator.prescoreCoralL1HeightMeters);
        break;
      case L2:
        requestLevel(level, Constants.Elevator.prescoreCoralL2HeightMeters);
        break;
      case L3:
        requestLevel(level, Constants.Elevator.prescoreCoralL3HeightMeters);
        break;
      case L4:
        requestLevel(level, Constants.Elevator.prescoreCoralL4HeightMeters);
        break;
    }
  }

  public void scoreCoral(Level level) {
    switch (level) {
      case L1:
        requestLevel(level, Constants.Scoring.scoreCoralL1HeightMeters);
        break;
      case L2:
        requestLevel(level, Constants.Scoring.scoreCoralL2HeightMeters);
        break;
      case L3:
        requestLevel(level, Constants.Scoring.scoreCoralL3HeightMeters);
        break;
      case L4:
        requestLevel(level, Constants.Scoring.scoreCoralL4HeightMeters);
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
    //TODO: Implement home logic
  }

  public void setNeutralMode(IdleMode mode) {
    io.stopElevator(mode);
  }

  public void safeBargeRetract() {
    requestedHeightMeters = Constants.Elevator.minElevatorSafeHeightMeters;
  }

  public void climbing() {
    requestedHeightMeters = 0;
    //needs work
  }
  //SETMANUALINTITIALIZATION
  //RequestAutomaticInitialization
  //home = intialization
  //set is now request
  //Configure hash to constants, so we don't have to change it after fault when we flash the robot
  //Hash is a mathmatical function you apply to a block of data, and the hashcode is produced by that function.
  //

  public void eject() {
    requestedHeightMeters = 0;
  }
  public void peformInitialization(){
    state = ElevatorStates.INITIALIZATIONPROCEDURE;
  }
}
