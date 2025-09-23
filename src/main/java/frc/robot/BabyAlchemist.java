package frc.robot;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.RobotMode;

public class BabyAlchemist {
  public static boolean init;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("kI");
  private static final LoggedTunableNumber iSat = new LoggedTunableNumber("iSat");
  private static final LoggedTunableNumber iZone = new LoggedTunableNumber("iZone");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("kD");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("kG");
  private static final LoggedTunableNumber acc = new LoggedTunableNumber("acc");
  private static final LoggedTunableNumber dec = new LoggedTunableNumber("dec");
  private static final LoggedTunableNumber vel = new LoggedTunableNumber("velocity");
  private static final LoggedTunableNumber setpoint = new LoggedTunableNumber("setpoint");

  public static Double run(Nitrate nitrate) {
    NitrateSettings settings;
    Double newPos = null;

    if (!init) {
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
      kP.initDefault(settings.getPIDSettings(PIDConfigSlot.kSlot0).getP().get());
      kI.initDefault(settings.getPIDSettings(PIDConfigSlot.kSlot0).getI().get());
      iSat.initDefault(settings.getPIDSettings(PIDConfigSlot.kSlot0).getISaturation().get());
      iZone.initDefault(settings.getPIDSettings(PIDConfigSlot.kSlot0).getIZone().get());
      kD.initDefault(settings.getPIDSettings(PIDConfigSlot.kSlot0).getD().get());
      kG.initDefault(
          settings.getPIDSettings(PIDConfigSlot.kSlot0).getGravitationalFeedforward().get());
      acc.initDefault(
          settings.getPIDSettings(PIDConfigSlot.kSlot0).getMotionProfileAccelLimit().get());
      dec.initDefault(
          settings.getPIDSettings(PIDConfigSlot.kSlot0).getMotionProfileDeaccelLimit().get());
      vel.initDefault(
          settings.getPIDSettings(PIDConfigSlot.kSlot0).getMotionProfileVelocityLimit().get());
      setpoint.initDefault(0);
      init = true;
    }

    settings = new NitrateSettings();
    if (kP.hasChanged(1)) {
      settings.setPIDSettings(new PIDSettings().setP(kP.get()), PIDConfigSlot.kSlot0);
    }
    if (kI.hasChanged(1)) {
      settings.setPIDSettings(new PIDSettings().setI(kI.get()), PIDConfigSlot.kSlot0);
    }
    if (iSat.hasChanged(1)) {
      settings.setPIDSettings(new PIDSettings().setISaturation(iSat.get()), PIDConfigSlot.kSlot0);
    }
    if (iZone.hasChanged(1)) {
      settings.setPIDSettings(new PIDSettings().setIZone(iZone.get()), PIDConfigSlot.kSlot0);
    }
    if (kD.hasChanged(1)) {
      settings.setPIDSettings(new PIDSettings().setD(kD.get()), PIDConfigSlot.kSlot0);
    }
    if (kG.hasChanged(1)) {
      settings.setPIDSettings(
          new PIDSettings().setGravitationalFeedforward(kG.get()), PIDConfigSlot.kSlot0);
    }
    if (acc.hasChanged(1)) {
      settings.setPIDSettings(
          new PIDSettings().setMotionProfileAccelLimit(acc.get()), PIDConfigSlot.kSlot0);
    }
    if (dec.hasChanged(1)) {
      settings.setPIDSettings(
          new PIDSettings().setMotionProfileDeaccelLimit(dec.get()), PIDConfigSlot.kSlot0);
    }
    if (vel.hasChanged(1)) {
      settings.setPIDSettings(
          new PIDSettings().setMotionProfileVelocityLimit(vel.get()), PIDConfigSlot.kSlot0);
    }
    if (setpoint.hasChanged(1)) {
      newPos = setpoint.get();
    }

    if (!settings.isEmpty() && (Constants.currentMode == RobotMode.REAL)) {
      settings.setEphemeral(true);  // avoid wear of the Nitrate flash
      NitrateSettings status = nitrate.setSettings(settings, 0.02, 5);
      if (!status.isEmpty()) {
        DriverStation.reportError(
            "Nitrate " + nitrate.getAddress().getDeviceId() + " did not receive settings", false);
      }
    }
    return newPos;
  }
}
