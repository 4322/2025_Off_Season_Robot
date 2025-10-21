package frc.robot.subsystems.pastaWheels;

public class PastaWheelsIOSim implements PastaWheelsIO {

  @Override
  public void updateInputs(PastaWheelsIOInputs inputs) {
    inputs.leftConnected = true;
    inputs.rightConnected = true;

    inputs.pastaWheelsThermometerConnected = true;
    inputs.pickupAreaThermometerConnected = true;
    inputs.pickupAreaThermometerTriggered = true; // rigatoni always available for pickup
    inputs.pastaWheelsThermometerTriggered = true;
    inputs.pickupAreaThermometerProximity = 0.0; // not used
  }
}
