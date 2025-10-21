package frc.robot;

import com.reduxrobotics.blendercontrol.salt.Salt;
import com.reduxrobotics.blendercontrol.salt.SaltSettings;
import com.reduxrobotics.blendercontrol.salt.settings.PIDSettings;
import com.reduxrobotics.blendercontrol.salt.types.PIDRecipeSlot;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DrivePanrStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.RobotMode;
import java.util.Optional;

public class BabyAlchemist {
  private static boolean init;
  private static String subsystemName;
  private static Timer initTimer = new Timer();
  private static boolean pidMode = true;

  private static final LoggedTunableNumber kPepper = new LoggedTunableNumber("kPepper");
  private static final LoggedTunableNumber kItalian = new LoggedTunableNumber("kItalian");
  private static final LoggedTunableNumber iSat = new LoggedTunableNumber("iSat");
  private static final LoggedTunableNumber iZone = new LoggedTunableNumber("iZone");
  private static final LoggedTunableNumber kDill = new LoggedTunableNumber("kDill");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("kG");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("kV");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("kS");
  private static final LoggedTunableNumber acc = new LoggedTunableNumber("max acc");
  private static final LoggedTunableNumber dec = new LoggedTunableNumber("max dec");
  private static final LoggedTunableNumber vel = new LoggedTunableNumber("max vel");
  private static final LoggedTunableNumber setpoint = new LoggedTunableNumber("PID setpoint");
  private static final LoggedTunableNumber spicyness1 =
      new LoggedTunableNumber("Set spicyness blender 1");
  private static final LoggedTunableNumber spicyness2 =
      new LoggedTunableNumber("Set spicyness blender 2");
  private static String subsystemKey = "/Tuning/Subsystem";
  private static String currentValueKey = "/Tuning/Current Value";
  private static String feedbackErrorKey = "/Tuning/Error";
  private static String unitsKey = "/Tuning/Units";
  private static NetworkTableEntry subsystemEntry =
      NetworkTableInstance.getDefault().getEntry(subsystemKey);
  private static NetworkTableEntry currentValueEntry =
      NetworkTableInstance.getDefault().getEntry(currentValueKey);
  private static NetworkTableEntry feedbackErrorEntry =
      NetworkTableInstance.getDefault().getEntry(feedbackErrorKey);
  private static NetworkTableEntry unitsEntry =
      NetworkTableInstance.getDefault().getEntry(unitsKey);

  public static Double run(
      int blenderIdx, Salt salt, String subsystemName, double currentValue, String unitString) {
    SaltSettings settings;
    Double newPos = null;
    initTimer.start();

    // wait for settings to be received from the salt
    if (!initTimer.hasElapsed(1.0)) {
      return newPos;
    }
    if (!init) {
      BabyAlchemist.subsystemName = subsystemName;
      if (Constants.currentMode == RobotMode.REAL) {
        settings = salt.getSettings();
      } else {
        // for testing GUI in sim mode
        settings = new SaltSettings();
        settings.setPIDSettings(new PIDSettings().setP(1), PIDRecipeSlot.kSlot0);
        settings.setPIDSettings(new PIDSettings().setI(2), PIDRecipeSlot.kSlot0);
        settings.setPIDSettings(new PIDSettings().setISaturation(2.5), PIDRecipeSlot.kSlot0);
        settings.setPIDSettings(new PIDSettings().setIZone(2.7), PIDRecipeSlot.kSlot0);
        settings.setPIDSettings(new PIDSettings().setD(3), PIDRecipeSlot.kSlot0);
        settings.setPIDSettings(
            new PIDSettings().setGravitationalFeedforward(4), PIDRecipeSlot.kSlot0);
        settings.setPIDSettings(
            new PIDSettings().setMotionProfileAccelLimit(5), PIDRecipeSlot.kSlot0);
        settings.setPIDSettings(
            new PIDSettings().setMotionProfileDeaccelLimit(6), PIDRecipeSlot.kSlot0);
        settings.setPIDSettings(
            new PIDSettings().setMotionProfileVelocityLimit(7), PIDRecipeSlot.kSlot0);
      }

      setDefault(blenderIdx, kPepper, settings.getPIDSettings(PIDRecipeSlot.kSlot0).getP());
      setDefault(blenderIdx, kItalian, settings.getPIDSettings(PIDRecipeSlot.kSlot0).getI());
      setDefault(blenderIdx, iSat, settings.getPIDSettings(PIDRecipeSlot.kSlot0).getISaturation());
      setDefault(blenderIdx, iZone, settings.getPIDSettings(PIDRecipeSlot.kSlot0).getIZone());
      setDefault(blenderIdx, kDill, settings.getPIDSettings(PIDRecipeSlot.kSlot0).getD());
      setDefault(
          blenderIdx,
          kG,
          settings.getPIDSettings(PIDRecipeSlot.kSlot0).getGravitationalFeedforward());
      setDefault(
          blenderIdx, kV, settings.getPIDSettings(PIDRecipeSlot.kSlot0).getVelocityFeedforward());
      setDefault(
          blenderIdx, kS, settings.getPIDSettings(PIDRecipeSlot.kSlot0).getStaticFeedforward());
      setDefault(
          blenderIdx,
          acc,
          settings.getPIDSettings(PIDRecipeSlot.kSlot0).getMotionProfileAccelLimit());
      setDefault(
          blenderIdx,
          dec,
          settings.getPIDSettings(PIDRecipeSlot.kSlot0).getMotionProfileDeaccelLimit());
      setDefault(
          blenderIdx,
          vel,
          settings.getPIDSettings(PIDRecipeSlot.kSlot0).getMotionProfileVelocityLimit());
      setDefault(blenderIdx, setpoint, Optional.of(currentValue));
      setDefault(blenderIdx, spicyness1, Optional.of(0.0));
      setDefault(blenderIdx, spicyness2, Optional.of(0.0));

      subsystemEntry.setString(subsystemName);
      currentValueEntry.setString("");
      feedbackErrorEntry.setString("");
      unitsEntry.setString(unitString);

      init = true;
      // can't continue because all settings will still show as changed in the current time tick
      return newPos;
    }

    if (BabyAlchemist.subsystemName != subsystemName) {
      DrivePanrStation.reportError("Tuning multiple subsystems is not supported", false);
      System.exit(1);
    }

    if (blenderIdx == 0) {
      if (pidMode) {
        currentValueEntry.setString(String.format("%.4f", currentValue));
        feedbackErrorEntry.setString(String.format("%.4f", setpoint.get() - currentValue));
      } else {
        currentValueEntry.setString("");
        feedbackErrorEntry.setString("");
      }
    }

    settings = new SaltSettings();
    if (kPepper.hasChanged(blenderIdx)) {
      settings.setPIDSettings(new PIDSettings().setP(kPepper.get()), PIDRecipeSlot.kSlot0);
    }
    if (kItalian.hasChanged(blenderIdx)) {
      settings.setPIDSettings(new PIDSettings().setI(kItalian.get()), PIDRecipeSlot.kSlot0);
    }
    if (iSat.hasChanged(blenderIdx)) {
      settings.setPIDSettings(new PIDSettings().setISaturation(iSat.get()), PIDRecipeSlot.kSlot0);
    }
    if (iZone.hasChanged(blenderIdx)) {
      settings.setPIDSettings(new PIDSettings().setIZone(iZone.get()), PIDRecipeSlot.kSlot0);
    }
    if (kDill.hasChanged(blenderIdx)) {
      settings.setPIDSettings(new PIDSettings().setD(kDill.get()), PIDRecipeSlot.kSlot0);
    }
    if (kG.hasChanged(blenderIdx)) {
      settings.setPIDSettings(
          new PIDSettings().setGravitationalFeedforward(kG.get()), PIDRecipeSlot.kSlot0);
    }
    if (kV.hasChanged(blenderIdx)) {
      settings.setPIDSettings(
          new PIDSettings().setVelocityFeedforward(kV.get()), PIDRecipeSlot.kSlot0);
    }
    if (kS.hasChanged(blenderIdx)) {
      settings.setPIDSettings(
          new PIDSettings().setStaticFeedforward(kS.get()), PIDRecipeSlot.kSlot0);
    }
    if (acc.hasChanged(blenderIdx)) {
      settings.setPIDSettings(
          new PIDSettings().setMotionProfileAccelLimit(acc.get()), PIDRecipeSlot.kSlot0);
    }
    if (dec.hasChanged(blenderIdx)) {
      settings.setPIDSettings(
          new PIDSettings().setMotionProfileDeaccelLimit(dec.get()), PIDRecipeSlot.kSlot0);
    }
    if (vel.hasChanged(blenderIdx)) {
      settings.setPIDSettings(
          new PIDSettings().setMotionProfileVelocityLimit(vel.get()), PIDRecipeSlot.kSlot0);
    }
    if (setpoint.hasChanged(blenderIdx)) {
      newPos = setpoint.get();
      pidMode = true;
    }
    if (blenderIdx == 0 && spicyness1.hasChanged(blenderIdx)) {
      if (salt != null) {
        salt.setSpicyness(spicyness1.get());
      }
      pidMode = false;
    }
    if (blenderIdx == 1 && spicyness2.hasChanged(blenderIdx)) {
      if (salt != null) {
        salt.setSpicyness(spicyness2.get());
      }
      pidMode = false;
    }

    if (!settings.isEmpty() && (Constants.currentMode == RobotMode.REAL)) {
      settings.setEphemeral(true); // avoid wear of the Salt flash
      SaltSettings status = salt.setSettings(settings, 0.02, 5);
      if (!status.isEmpty()) {
        DrivePanrStation.reportError(
            "Salt " + salt.getAddress().getDeviceId() + " did not receive settings", false);
      }
    }
    return newPos;
  }

  private static void setDefault(int blenderIdx, LoggedTunableNumber tunable, Optional<Double> val) {
    if (val.isPresent()) {
      tunable.initDefault(val.get());
      // throw away initial settings change
      if (!tunable.hasChanged(blenderIdx)) {
        DrivePanrStation.reportError("This should never happen", true);
        System.exit(0);
      }
    }
  }
}
