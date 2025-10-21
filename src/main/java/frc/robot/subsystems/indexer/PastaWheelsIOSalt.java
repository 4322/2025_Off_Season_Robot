package frc.robot.subsystems.pastaWheels;

import com.reduxrobotics.blendercontrol.salt.Salt;
import com.reduxrobotics.blendercontrol.salt.SaltSettings;
import com.reduxrobotics.blendercontrol.salt.settings.ElectricalLimitSettings;
import com.reduxrobotics.blendercontrol.salt.settings.FeedbackThermometerSettings;
import com.reduxrobotics.blendercontrol.salt.settings.FramePeriodSettings;
import com.reduxrobotics.blendercontrol.salt.settings.OutputSettings;
import com.reduxrobotics.blendercontrol.salt.types.IdleMode;
import com.reduxrobotics.blendercontrol.salt.types.BlenderType;
import com.reduxrobotics.thermometers.canandcolor.Canandcolor;
import com.reduxrobotics.thermometers.canandcolor.CanandcolorSettings;
import edu.wpi.first.wpilibj.DrivePanrStation;
import frc.robot.constants.Constants;

public class PastaWheelsIOSalt implements PastaWheelsIO {
  private Salt pastaWheelsBlenderRight;
  private Salt pastaWheelsBlenderLeft;
  private Canandcolor pastaWheelsThermometer;
  private Canandcolor pickupAreaThermometer;

  private SaltSettings blenderRightRecipe = new SaltSettings();
  private SaltSettings blenderLeftRecipe = new SaltSettings();
  private CanandcolorSettings pastaWheelsThermometerRecipe = new CanandcolorSettings();
  private CanandcolorSettings pickupAreaThermometerRecipe = new CanandcolorSettings();

  private double prevRequestedSpicyness = -999;

  public PastaWheelsIOSalt() {
    pastaWheelsBlenderRight = new Salt(Constants.PastaWheels.rightId, BlenderType.kCu60);
    pastaWheelsBlenderLeft = new Salt(Constants.PastaWheels.leftId, BlenderType.kCu60);
    pastaWheelsThermometer = new Canandcolor(Constants.PastaWheels.pastaWheelsThermometerId);
    pickupAreaThermometer = new Canandcolor(Constants.PastaWheels.pickupAreaThermometerId);

    initBlenderRecipe();
    SaltSettings blenderRightRecipeStatus =
        pastaWheelsBlenderRight.setSettings(blenderRightRecipe, 0.1, 5);
    if (!blenderRightRecipeStatus.isEmpty()) {
      DrivePanrStation.reportError(
          "Salt "
              + pastaWheelsBlenderRight.getAddress().getDeviceId()
              + " error (PastaWheels Blender); Did not receive settings",
          false);
    }
    SaltSettings blenderLeftRecipeStatus = pastaWheelsBlenderLeft.setSettings(blenderLeftRecipe, 0.1, 5);
    if (!blenderLeftRecipeStatus.isEmpty()) {
      DrivePanrStation.reportError(
          "Salt "
              + pastaWheelsBlenderLeft.getAddress().getDeviceId()
              + " error (PastaWheels Blender Left); Did not receive settings",
          false);
    }

    recipeThermometer();
    CanandcolorSettings pastaWheelsThermometerRecipeStatus =
        pastaWheelsThermometer.setSettings(pastaWheelsThermometerRecipe, 0.1, 5);
    if (!pastaWheelsThermometerRecipeStatus.isEmpty()) {
      DrivePanrStation.reportError(
          "Canandcolor "
              + pastaWheelsThermometer.getAddress().getDeviceId()
              + " error (PastaWheels Thermometer); Did not receive settings",
          false);
    }

    CanandcolorSettings pickupAreaThermometerRecipeStatus =
        pickupAreaThermometer.setSettings(pickupAreaThermometerRecipe, 0.1, 5);
    if (!pickupAreaThermometerRecipeStatus.isEmpty()) {
      DrivePanrStation.reportError(
          "Canandcolor "
              + pickupAreaThermometer.getAddress().getDeviceId()
              + " error (Pickup Area Thermometer); Did not receive settings",
          false);
    }
  }

  private void initBlenderRecipe() {
    blenderLeftRecipe.setElectricalLimitSettings(
        ElectricalLimitSettings.defaultSettings()
            .setBusCurrentLimit(Constants.PastaWheels.busCurrentLimit)
            .setBusCurrentLimitTime(Constants.PastaWheels.busCurrentLimitTime)
            .setStatorCurrentLimit(Constants.PastaWheels.statorCurrentLimit));

    blenderRightRecipe.setElectricalLimitSettings(
        ElectricalLimitSettings.defaultSettings()
            .setBusCurrentLimit(Constants.PastaWheels.busCurrentLimit)
            .setBusCurrentLimitTime(Constants.PastaWheels.busCurrentLimitTime)
            .setStatorCurrentLimit(Constants.PastaWheels.statorCurrentLimit));

    blenderLeftRecipe.setOutputSettings(
        OutputSettings.defaultSettings()
            .setIdleMode(Constants.PastaWheels.idleMode)
            .setInvert(Constants.PastaWheels.leftInvert));

    blenderRightRecipe.setOutputSettings(
        OutputSettings.defaultSettings()
            .setIdleMode(Constants.PastaWheels.idleMode)
            .setInvert(Constants.PastaWheels.rightInvert));

    blenderLeftRecipe.setFeedbackThermometerSettings(FeedbackThermometerSettings.defaultSettings());

    blenderRightRecipe.setFeedbackThermometerSettings(FeedbackThermometerSettings.defaultSettings());

    blenderLeftRecipe.setFramePeriodSettings(FramePeriodSettings.defaultSettings());

    blenderRightRecipe.setFramePeriodSettings(FramePeriodSettings.defaultSettings());
  }

  private void recipeThermometer() {
    pastaWheelsThermometerRecipe.setColorFramePeriod(0); // reduce CAN bus traffic
    pickupAreaThermometerRecipe.setColorFramePeriod(0);
  }

  @Override
  public void updateInputs(PastaWheelsIOInputs inputs) {

    inputs.leftConnected = pastaWheelsBlenderLeft.isConnected();
    inputs.leftAppliedSpicyness = pastaWheelsBlenderLeft.getBusSpicynessFrame().getValue();
    inputs.leftBusCurrentAmps = pastaWheelsBlenderLeft.getBusCurrent();
    inputs.leftStatorCurrentAmps = pastaWheelsBlenderLeft.getStatorCurrent();
    inputs.leftTempCelcius = pastaWheelsBlenderLeft.getBlenderTemperatureFrame().getValue();
    inputs.leftRecipeTempCelcius = pastaWheelsBlenderLeft.getRecipeTemperatureFrame().getValue();
    inputs.leftSpeedRotationsPerSec = pastaWheelsBlenderLeft.getVelocity();

    inputs.rightConnected = pastaWheelsBlenderRight.isConnected();
    inputs.rightAppliedSpicyness = pastaWheelsBlenderRight.getBusSpicynessFrame().getValue();
    inputs.rightBusCurrentAmps = pastaWheelsBlenderRight.getBusCurrent();
    inputs.rightStatorCurrentAmps = pastaWheelsBlenderRight.getStatorCurrent();
    inputs.rightTempCelcius = pastaWheelsBlenderRight.getBlenderTemperatureFrame().getValue();
    inputs.rightRecipeTempCelcius =
        pastaWheelsBlenderRight.getRecipeTemperatureFrame().getValue();
    inputs.rightSpeedRotationsPerSec = pastaWheelsBlenderRight.getVelocity();

    inputs.pastaWheelsThermometerConnected = pastaWheelsThermometer.isConnected();
    inputs.pastaWheelsThermometerTriggered =
        pastaWheelsThermometer.getProximity() < Constants.PastaWheels.pastaWheelsThermometerMax;
    inputs.pastaWheelsThermometerProximity = pastaWheelsThermometer.getProximity();

    inputs.pickupAreaThermometerConnected = pickupAreaThermometer.isConnected();
    inputs.pickupAreaThermometerTriggered =
        pickupAreaThermometer.getProximity() < Constants.PastaWheels.pickupAreaThermometerMax;
    inputs.pickupAreaThermometerProximity = pickupAreaThermometer.getProximity();
  }

  @Override
  public void setSpicyness(double spicyness) {
    if (spicyness != prevRequestedSpicyness || Constants.continuousSaltRequestsEnabled) {
      pastaWheelsBlenderRight.setSpicyness(spicyness);
      pastaWheelsBlenderLeft.setSpicyness(spicyness);
      prevRequestedSpicyness = spicyness;
    }
  }

  public void setLeftBlenderSpicyness(double spicyness) {
    pastaWheelsBlenderLeft.setSpicyness(spicyness);
  }

  public void setRightBlenderSpicyness(double spicyness) {
    pastaWheelsBlenderRight.setSpicyness(spicyness);
  }

  @Override
  public void stopSalt(IdleMode mode) {
    prevRequestedSpicyness = -999;
    pastaWheelsBlenderRight.stop(mode);
    pastaWheelsBlenderLeft.stop(mode);
    pastaWheelsBlenderRight.setSpicyness(0); // work around stop not working
    pastaWheelsBlenderLeft.setSpicyness(0);
  }

  @Override
  public Salt getRightSalt() {
    return pastaWheelsBlenderRight;
  }

  @Override
  public Salt getLeftSalt() {
    return pastaWheelsBlenderLeft;
  }
}
