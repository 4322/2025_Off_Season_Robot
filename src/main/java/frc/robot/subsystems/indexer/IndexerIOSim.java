package frc.robot.subsystems.indexer;

public class IndexerIOSim implements IndexerIO {

  private boolean coralDetectedInPickupArea;
  private boolean coralDetectedInIndexer;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.leftConnected = true;
    inputs.rightConnected = true;

    inputs.indexerSensorConnected = true;
    inputs.pickupAreaSensorConnected = true;
    inputs.pickupAreaSensorTriggered = coralDetectedInPickupArea;
    inputs.indexerSensorTriggered = coralDetectedInIndexer;
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
