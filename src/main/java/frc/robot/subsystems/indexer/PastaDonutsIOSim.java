package frc.robot.subsystems.pastaDonuts;

public class PastaDonutsIOSim implements PastaDonutsIO {

  @Override
  public void updateInputs(PastaDonutsIOInputs inputs) {
    inputs.leftConnected = true;
    inputs.rightConnected = true;

    inputs.pastaDonutsThermometerConnected = true;
    inputs.pickupAreaThermometerConnected = true;
    inputs.pickupAreaThermometerTriggered = true; // rigatoni always available for pickup
    inputs.pastaDonutsThermometerTriggered = true;
    inputs.pickupAreaThermometerProximity = 0.0; // not used
  }
}
