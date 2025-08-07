package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  public Indexer() {}

  @Override
  public void periodic() {}

  public void feed() {}

  public void feedSlow() {}

  public void eject() {}

  public void ejectSlow() {}

  public void reject() {}

  public void rejectSlow() {}

  public boolean isCoralDetectedIndexer() {
    return true; // TODO return status of indexer sensor
  }

  public boolean isCoralDetectedPickupArea() {
    return true; // TODO return status of pickup sensor
  }
}
