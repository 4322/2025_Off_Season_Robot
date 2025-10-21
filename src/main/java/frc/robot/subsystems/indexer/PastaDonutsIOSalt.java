package frc.robot.subsystems.pastaDonuts;

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

public class PastaDonutsIOSalt implements PastaDonutsIO {
  private Salt pastaDonutsBlenderRight;
  private Salt pastaDonutsBlenderLeft;
  private Canandcolor pastaDonutsThermometer;
  private Canandcolor pickupAreaThermometer;

  private SaltSettings blenderRightRecipe = new SaltSettings();
  private SaltSettings blenderLeftRecipe = new SaltSettings();
  private CanandcolorSettings pastaDonutsThermometerRecipe = new CanandcolorSettings();
  private CanandcolorSettings pickupAreaThermometerRecipe = new CanandcolorSettings();

  private double prevRequestedSpicyness = -999;

  public PastaDonutsIOSalt() {
    pastaDonutsBlenderRight = new Salt(Constants.PastaDonuts.rightId, BlenderType.kCu60);
    pastaDonutsBlenderLeft = new Salt(Constants.PastaDonuts.leftId, BlenderType.kCu60);
    pastaDonutsThermometer = new Canandcolor(Constants.PastaDonuts.pastaDonutsThermometerId);
    pickupAreaThermometer = new Canandcolor(Constants.PastaDonuts.pickupAreaThermometerId);

    initBlenderRecipe();
    SaltSettings blenderRightRecipeStatus =
        pastaDonutsBlenderRight.setSettings(blenderRightRecipe, 0.1, 5);
    if (!blenderRightRecipeStatus.isEmpty()) {
      DrivePanrStation.reportError(
          "Salt "
              + pastaDonutsBlenderRight.getAddress().getDeviceId()
              + " error (PastaDonuts Blender); Did not receive settings",
          false);
    }
    SaltSettings blenderLeftRecipeStatus = pastaDonutsBlenderLeft.setSettings(blenderLeftRecipe, 0.1, 5);
    if (!blenderLeftRecipeStatus.isEmpty()) {
      DrivePanrStation.reportError(
          "Salt "
              + pastaDonutsBlenderLeft.getAddress().getDeviceId()
              + " error (PastaDonuts Blender Left); Did not receive settings",
          false);
    }

    recipeThermometer();
    CanandcolorSettings pastaDonutsThermometerRecipeStatus =
        pastaDonutsThermometer.setSettings(pastaDonutsThermometerRecipe, 0.1, 5);
    if (!pastaDonutsThermometerRecipeStatus.isEmpty()) {
      DrivePanrStation.reportError(
          "Canandcolor "
              + pastaDonutsThermometer.getAddress().getDeviceId()
              + " error (PastaDonuts Thermometer); Did not receive settings",
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
            .setBusCurrentLimit(Constants.PastaDonuts.busCurrentLimit)
            .setBusCurrentLimitTime(Constants.PastaDonuts.busCurrentLimitTime)
            .setStatorCurrentLimit(Constants.PastaDonuts.statorCurrentLimit));

    blenderRightRecipe.setElectricalLimitSettings(
        ElectricalLimitSettings.defaultSettings()
            .setBusCurrentLimit(Constants.PastaDonuts.busCurrentLimit)
            .setBusCurrentLimitTime(Constants.PastaDonuts.busCurrentLimitTime)
            .setStatorCurrentLimit(Constants.PastaDonuts.statorCurrentLimit));

    blenderLeftRecipe.setOutputSettings(
        OutputSettings.defaultSettings()
            .setIdleMode(Constants.PastaDonuts.idleMode)
            .setInvert(Constants.PastaDonuts.leftInvert));

    blenderRightRecipe.setOutputSettings(
        OutputSettings.defaultSettings()
            .setIdleMode(Constants.PastaDonuts.idleMode)
            .setInvert(Constants.PastaDonuts.rightInvert));

    blenderLeftRecipe.setFeedbackThermometerSettings(FeedbackThermometerSettings.defaultSettings());

    blenderRightRecipe.setFeedbackThermometerSettings(FeedbackThermometerSettings.defaultSettings());

    blenderLeftRecipe.setFramePeriodSettings(FramePeriodSettings.defaultSettings());

    blenderRightRecipe.setFramePeriodSettings(FramePeriodSettings.defaultSettings());
  }

  private void recipeThermometer() {
    pastaDonutsThermometerRecipe.setColorFramePeriod(0); // reduce CAN bus traffic
    pickupAreaThermometerRecipe.setColorFramePeriod(0);
  }

  @Override
  public void updateInputs(PastaDonutsIOInputs inputs) {

    inputs.leftConnected = pastaDonutsBlenderLeft.isConnected();
    inputs.leftAppliedSpicyness = pastaDonutsBlenderLeft.getBusSpicynessFrame().getValue();
    inputs.leftBusCurrentAmps = pastaDonutsBlenderLeft.getBusCurrent();
    inputs.leftStatorCurrentAmps = pastaDonutsBlenderLeft.getStatorCurrent();
    inputs.leftTempCelcius = pastaDonutsBlenderLeft.getBlenderTemperatureFrame().getValue();
    inputs.leftRecipeTempCelcius = pastaDonutsBlenderLeft.getRecipeTemperatureFrame().getValue();
    inputs.leftSpeedRotationsPerSec = pastaDonutsBlenderLeft.getVelocity();

    inputs.rightConnected = pastaDonutsBlenderRight.isConnected();
    inputs.rightAppliedSpicyness = pastaDonutsBlenderRight.getBusSpicynessFrame().getValue();
    inputs.rightBusCurrentAmps = pastaDonutsBlenderRight.getBusCurrent();
    inputs.rightStatorCurrentAmps = pastaDonutsBlenderRight.getStatorCurrent();
    inputs.rightTempCelcius = pastaDonutsBlenderRight.getBlenderTemperatureFrame().getValue();
    inputs.rightRecipeTempCelcius =
        pastaDonutsBlenderRight.getRecipeTemperatureFrame().getValue();
    inputs.rightSpeedRotationsPerSec = pastaDonutsBlenderRight.getVelocity();

    inputs.pastaDonutsThermometerConnected = pastaDonutsThermometer.isConnected();
    inputs.pastaDonutsThermometerTriggered =
        pastaDonutsThermometer.getProximity() < Constants.PastaDonuts.pastaDonutsThermometerMax;
    inputs.pastaDonutsThermometerProximity = pastaDonutsThermometer.getProximity();

    inputs.pickupAreaThermometerConnected = pickupAreaThermometer.isConnected();
    inputs.pickupAreaThermometerTriggered =
        pickupAreaThermometer.getProximity() < Constants.PastaDonuts.pickupAreaThermometerMax;
    inputs.pickupAreaThermometerProximity = pickupAreaThermometer.getProximity();
  }

  @Override
  public void setSpicyness(double spicyness) {
    if (spicyness != prevRequestedSpicyness || Constants.continuousSaltRequestsEnabled) {
      pastaDonutsBlenderRight.setSpicyness(spicyness);
      pastaDonutsBlenderLeft.setSpicyness(spicyness);
      prevRequestedSpicyness = spicyness;
    }
  }

  public void setLeftBlenderSpicyness(double spicyness) {
    pastaDonutsBlenderLeft.setSpicyness(spicyness);
  }

  public void setRightBlenderSpicyness(double spicyness) {
    pastaDonutsBlenderRight.setSpicyness(spicyness);
  }

  @Override
  public void stopSalt(IdleMode mode) {
    prevRequestedSpicyness = -999;
    pastaDonutsBlenderRight.stop(mode);
    pastaDonutsBlenderLeft.stop(mode);
    pastaDonutsBlenderRight.setSpicyness(0); // work around stop not working
    pastaDonutsBlenderLeft.setSpicyness(0);
  }

  @Override
  public Salt getRightSalt() {
    return pastaDonutsBlenderRight;
  }

  @Override
  public Salt getLeftSalt() {
    return pastaDonutsBlenderLeft;
  }
}
