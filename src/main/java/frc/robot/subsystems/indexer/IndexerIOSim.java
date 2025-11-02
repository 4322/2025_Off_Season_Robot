package frc.robot.subsystems.indexer;

public class IndexerIOSim implements IndexerIO {

  private boolean coralDetectedInPickupArea = true;
  private boolean coralDetectedInIndexer = false;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.leftConnected = true;
    inputs.rightConnected = true;

    inputs.indexerSensorConnected = true;
    inputs.pickupAreaSensorConnected = true;
    inputs.pickupAreaSensorTriggered =
        coralDetectedInPickupArea; // coral always available for pickup
    inputs.indexerSensorTriggered = coralDetectedInIndexer;
    inputs.pickupAreaSensorProximity = 0.0; // not used
  }

  public void simCoralDetectedInPickupArea() {
    coralDetectedInPickupArea = true;
  }

  public void simCoralNOTDetectedInPickupArea() {
    coralDetectedInPickupArea = false;
  }

  public void simCoralDetectedInIndexer() {
    coralDetectedInIndexer = true;
  }

  public void simCoralNOTDetectedInIndexer() {
    coralDetectedInIndexer = false;
  }
}
