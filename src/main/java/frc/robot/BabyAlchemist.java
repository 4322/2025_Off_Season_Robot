package frc.robot;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.RobotMode;
import java.util.Optional;

public class BabyAlchemist {
  private static boolean init;
  private static String subsystemName;
  private static Timer initTimer = new Timer();
  private static boolean pidMode = true;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("kI");
  private static final LoggedTunableNumber iSat = new LoggedTunableNumber("iSat");
  private static final LoggedTunableNumber iZone = new LoggedTunableNumber("iZone");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("kD");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("kG");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("kV");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("kS");
  private static final LoggedTunableNumber acc = new LoggedTunableNumber("max acc");
  private static final LoggedTunableNumber dec = new LoggedTunableNumber("max dec");
  private static final LoggedTunableNumber vel = new LoggedTunableNumber("max vel");
  private static final LoggedTunableNumber setpoint = new LoggedTunableNumber("PID setpoint");
  private static final LoggedTunableNumber voltage1 =
      new LoggedTunableNumber("Set voltage motor 1");
  private static final LoggedTunableNumber voltage2 =
      new LoggedTunableNumber("Set voltage motor 2");
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
      int motorIdx, Nitrate nitrate, String subsystemName, double currentValue, String unitString) {
    NitrateSettings settings;
    Double newPos = null;
    initTimer.start();

    // wait for settings to be received from the nitrate
    if (!initTimer.hasElapsed(1.0)) {
      return newPos;
    }
    if (!init) {
      BabyAlchemist.subsystemName = subsystemName;
      if (Constants.currentMode == RobotMode.REAL) {
        settings = nitrate.getSettings();
      } else {
        // for testing GUI in sim mode
        settings = new NitrateSettings();
        settings.setPIDSettings(new PIDSettings().setP(1), PIDConfigSlot.kSlot0);
        settings.setPIDSettings(new PIDSettings().setI(2), PIDConfigSlot.kSlot0);
        settings.setPIDSettings(new PIDSettings().setISaturation(2.5), PIDConfigSlot.kSlot0);
        settings.setPIDSettings(new PIDSettings().setIZone(2.7), PIDConfigSlot.kSlot0);
        settings.setPIDSettings(new PIDSettings().setD(3), PIDConfigSlot.kSlot0);
        settings.setPIDSettings(
            new PIDSettings().setGravitationalFeedforward(4), PIDConfigSlot.kSlot0);
        settings.setPIDSettings(
            new PIDSettings().setMotionProfileAccelLimit(5), PIDConfigSlot.kSlot0);
        settings.setPIDSettings(
            new PIDSettings().setMotionProfileDeaccelLimit(6), PIDConfigSlot.kSlot0);
        settings.setPIDSettings(
            new PIDSettings().setMotionProfileVelocityLimit(7), PIDConfigSlot.kSlot0);
      }

      setDefault(motorIdx, kP, settings.getPIDSettings(PIDConfigSlot.kSlot0).getP());
      setDefault(motorIdx, kI, settings.getPIDSettings(PIDConfigSlot.kSlot0).getI());
      setDefault(motorIdx, iSat, settings.getPIDSettings(PIDConfigSlot.kSlot0).getISaturation());
      setDefault(motorIdx, iZone, settings.getPIDSettings(PIDConfigSlot.kSlot0).getIZone());
      setDefault(motorIdx, kD, settings.getPIDSettings(PIDConfigSlot.kSlot0).getD());
      setDefault(
          motorIdx,
          kG,
          settings.getPIDSettings(PIDConfigSlot.kSlot0).getGravitationalFeedforward());
      setDefault(
          motorIdx, kV, settings.getPIDSettings(PIDConfigSlot.kSlot0).getVelocityFeedforward());
      setDefault(
          motorIdx, kS, settings.getPIDSettings(PIDConfigSlot.kSlot0).getStaticFeedforward());
      setDefault(
          motorIdx,
          acc,
          settings.getPIDSettings(PIDConfigSlot.kSlot0).getMotionProfileAccelLimit());
      setDefault(
          motorIdx,
          dec,
          settings.getPIDSettings(PIDConfigSlot.kSlot0).getMotionProfileDeaccelLimit());
      setDefault(
          motorIdx,
          vel,
          settings.getPIDSettings(PIDConfigSlot.kSlot0).getMotionProfileVelocityLimit());
      setDefault(motorIdx, setpoint, Optional.of(currentValue));
      setDefault(motorIdx, voltage1, Optional.of(0.0));
      setDefault(motorIdx, voltage2, Optional.of(0.0));

      subsystemEntry.setString(subsystemName);
      currentValueEntry.setString("");
      feedbackErrorEntry.setString("");
      unitsEntry.setString(unitString);

      init = true;
      // can't continue because all settings will still show as changed in the current time tick
      return newPos;
    }

    if (BabyAlchemist.subsystemName != subsystemName) {
      DriverStation.reportError("Tuning multiple subsystems is not supported", false);
      System.exit(1);
    }

    if (motorIdx == 0) {
      if (pidMode) {
        currentValueEntry.setString(String.format("%.4f", currentValue));
        feedbackErrorEntry.setString(String.format("%.4f", setpoint.get() - currentValue));
      } else {
        currentValueEntry.setString("");
        feedbackErrorEntry.setString("");
      }
    }

    settings = new NitrateSettings();
    if (kP.hasChanged(motorIdx)) {
      settings.setPIDSettings(new PIDSettings().setP(kP.get()), PIDConfigSlot.kSlot0);
    }
    if (kI.hasChanged(motorIdx)) {
      settings.setPIDSettings(new PIDSettings().setI(kI.get()), PIDConfigSlot.kSlot0);
    }
    if (iSat.hasChanged(motorIdx)) {
      settings.setPIDSettings(new PIDSettings().setISaturation(iSat.get()), PIDConfigSlot.kSlot0);
    }
    if (iZone.hasChanged(motorIdx)) {
      settings.setPIDSettings(new PIDSettings().setIZone(iZone.get()), PIDConfigSlot.kSlot0);
    }
    if (kD.hasChanged(motorIdx)) {
      settings.setPIDSettings(new PIDSettings().setD(kD.get()), PIDConfigSlot.kSlot0);
    }
    if (kG.hasChanged(motorIdx)) {
      settings.setPIDSettings(
          new PIDSettings().setGravitationalFeedforward(kG.get()), PIDConfigSlot.kSlot0);
    }
    if (kV.hasChanged(motorIdx)) {
      settings.setPIDSettings(
          new PIDSettings().setVelocityFeedforward(kV.get()), PIDConfigSlot.kSlot0);
    }
    if (kS.hasChanged(motorIdx)) {
      settings.setPIDSettings(
          new PIDSettings().setStaticFeedforward(kS.get()), PIDConfigSlot.kSlot0);
    }
    if (acc.hasChanged(motorIdx)) {
      settings.setPIDSettings(
          new PIDSettings().setMotionProfileAccelLimit(acc.get()), PIDConfigSlot.kSlot0);
    }
    if (dec.hasChanged(motorIdx)) {
      settings.setPIDSettings(
          new PIDSettings().setMotionProfileDeaccelLimit(dec.get()), PIDConfigSlot.kSlot0);
    }
    if (vel.hasChanged(motorIdx)) {
      settings.setPIDSettings(
          new PIDSettings().setMotionProfileVelocityLimit(vel.get()), PIDConfigSlot.kSlot0);
    }
    if (setpoint.hasChanged(motorIdx)) {
      newPos = setpoint.get();
      pidMode = true;
    }
    if (motorIdx == 0 && voltage1.hasChanged(motorIdx)) {
      if (nitrate != null) {
        nitrate.setVoltage(voltage1.get());
      }
      pidMode = false;
    }
    if (motorIdx == 1 && voltage2.hasChanged(motorIdx)) {
      if (nitrate != null) {
        nitrate.setVoltage(voltage2.get());
      }
      pidMode = false;
    }

    if (!settings.isEmpty() && (Constants.currentMode == RobotMode.REAL)) {
      settings.setEphemeral(true); // avoid wear of the Nitrate flash
      NitrateSettings status = nitrate.setSettings(settings, 0.02, 5);
      if (!status.isEmpty()) {
        DriverStation.reportError(
            "Nitrate " + nitrate.getAddress().getDeviceId() + " did not receive settings", false);
      }
    }
    return newPos;
  }

  private static void setDefault(int motorIdx, LoggedTunableNumber tunable, Optional<Double> val) {
    if (val.isPresent()) {
      tunable.initDefault(val.get());
      // throw away initial settings change
      if (!tunable.hasChanged(motorIdx)) {
        System.exit(0); // will never happen
      }
    }
  }
}
