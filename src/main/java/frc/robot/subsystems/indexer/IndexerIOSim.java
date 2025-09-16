package frc.robot.subsystems.indexer;

public class IndexerIOSim implements IndexerIO {

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerMotorLeftConnected = true;
    inputs.indexerMotorRightConnected = true;

    inputs.indexerSensorConnected = true;
    inputs.pickupAreaSensorConnected = true;
    inputs.pickupAreaSensorTriggered = true; // coral always available for pickup
    inputs.pickupAreaSensorProximity = 0.0; // not used
  }
}
