package frc.robot.subsystems.endEffector;

import frc.robot.constants.Constants;

public class EndEffectorIOSim implements EndEffectorIO {
  // Preload for beginning of auto
  private boolean coralDetected = false;
  private double sensorReading = Constants.EndEffector.coralProximityThreshold - 0.1;

  private boolean algaeDetected = false;

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.isCoralProximityDetected = coralDetected;
    inputs.isAlgaeProximityDetected = algaeDetected;
    inputs.sensorProximity = sensorReading;
  }


  public void simCoralHeld() {
    coralDetected = true;
    sensorReading = Constants.EndEffector.coralProximityThreshold - 0.1;
  }


  public void simAlgaeHeld() {
    algaeDetected = true;
    sensorReading = Constants.EndEffector.algaeProximityThresholdIntake - 0.1;
  }

  
  public void simCoralReleased() {
    coralDetected = false;
    sensorReading = Constants.EndEffector.coralProximityThreshold + 0.1;
  }

  public void simAlgaeReleased() {
    algaeDetected = false;
    sensorReading = Constants.EndEffector.algaeProximityThresholdIntake + 0.1;
  }
}
