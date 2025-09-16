package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class EndEffectorIOSim implements EndEffectorIO {

  LoggedNetworkBoolean coralDetected =
      new LoggedNetworkBoolean("SimParameters/SetCoralDetected", false);
  LoggedNetworkBoolean algaeDetected =
      new LoggedNetworkBoolean("SimParameters/SetAlgaeDetected", false);

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.isCoralProximityDetected = coralDetected.get();
    inputs.isAlgaeProximityDetected = algaeDetected.get();
  }
}
