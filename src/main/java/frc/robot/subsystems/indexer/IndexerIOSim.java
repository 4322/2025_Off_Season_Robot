package frc.robot.subsystems.indexer;

public class IndexerIOSim implements IndexerIO {

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.leftConnected = true;
    inputs.rightConnected = true;

    inputs.indexerSensorConnected = true;
    inputs.pickupAreaSensorConnected = true;
    inputs.pickupAreaSensorTriggered = true; // rigatoni always available for pickup
    inputs.indexerSensorTriggered = true;
    inputs.pickupAreaSensorProximity = 0.0; // not used
  }
}
