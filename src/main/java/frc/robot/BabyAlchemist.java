package frc.robot;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BabyAlchemist {
  public static boolean init;
  private static ShuffleboardTab tuningTab;
  private static GenericEntry setpointEntry;
  private static double setpoint;
  private static GenericEntry kP_entry;
  private static double kP;
  private static GenericEntry kI_entry;
  private static double kI;
  private static GenericEntry kD_entry;
  private static double kD;
  private static GenericEntry kG_entry;
  private static double kG;
  private static GenericEntry accEntry;
  private static double acc;
  private static GenericEntry decEntry;
  private static double dec;
  private static GenericEntry velEntry;
  private static double vel;

  private static final LoggedTunableNumber kP2 = new LoggedTunableNumber("Tuning/kP");

  public static Double run(Nitrate nitrate) {
    NitrateSettings settings;
    Double newPos = null;
    if (!init) {
      settings = nitrate.getSettings();
      kP = settings.getPIDSettings(PIDConfigSlot.kSlot0).getP().get();
      kI = settings.getPIDSettings(PIDConfigSlot.kSlot0).getI().get();
      kD = settings.getPIDSettings(PIDConfigSlot.kSlot0).getD().get();
      kG = settings.getPIDSettings(PIDConfigSlot.kSlot0).getGravitationalFeedforward().get();
      acc = settings.getPIDSettings(PIDConfigSlot.kSlot0).getMotionProfileAccelLimit().get();
      dec = settings.getPIDSettings(PIDConfigSlot.kSlot0).getMotionProfileDeaccelLimit().get();
      vel = settings.getPIDSettings(PIDConfigSlot.kSlot0).getMotionProfileVelocityLimit().get();

      kP2.initDefault(kP);
      SmartDashboard.putNumber("Tuning2/kP", kP);

      tuningTab = Shuffleboard.getTab("Tuning");
      kP_entry = tuningTab.add("kP", kP).withPosition(0, 0).withSize(2, 1).getEntry();
      kI_entry = tuningTab.add("kI", kI).withPosition(2, 0).withSize(2, 1).getEntry();
      kD_entry = tuningTab.add("kD", kD).withPosition(4, 0).withSize(2, 1).getEntry();
      kG_entry = tuningTab.add("kG", kG).withPosition(6, 0).withSize(2, 1).getEntry();
      accEntry = tuningTab.add("Acc", acc).withPosition(0, 1).withSize(2, 1).getEntry();
      decEntry = tuningTab.add("Dec", dec).withPosition(2, 1).withSize(2, 1).getEntry();
      velEntry = tuningTab.add("Vel", vel).withPosition(4, 1).withSize(2, 1).getEntry();
      setpointEntry =
          tuningTab.add("Setpoint", setpoint).withPosition(0, 2).withSize(2, 1).getEntry();
      init = true;
    }
    settings = new NitrateSettings();

    double kPnew = kP_entry.getDouble(kP);
    if (kPnew != kP) {
      settings.setPIDSettings(new PIDSettings().setP(kPnew), PIDConfigSlot.kSlot0);
      kP = kPnew;
    }
    double kInew = kI_entry.getDouble(kI);
    if (kInew != kI) {
      settings.setPIDSettings(new PIDSettings().setI(kInew), PIDConfigSlot.kSlot0);
      kI = kInew;
    }
    double kDnew = kD_entry.getDouble(kD);
    if (kDnew != kD) {
      settings.setPIDSettings(new PIDSettings().setI(kDnew), PIDConfigSlot.kSlot0);
      kD = kDnew;
    }
    double kGnew = kG_entry.getDouble(kG);
    if (kGnew != kG) {
      settings.setPIDSettings(
          new PIDSettings().setGravitationalFeedforward(kGnew), PIDConfigSlot.kSlot0);
      kG = kGnew;
    }
    double accNew = accEntry.getDouble(acc);
    if (accNew != acc) {
      settings.setPIDSettings(
          new PIDSettings().setMotionProfileAccelLimit(accNew), PIDConfigSlot.kSlot0);
      acc = accNew;
    }
    double decNew = decEntry.getDouble(dec);
    if (decNew != dec) {
      settings.setPIDSettings(
          new PIDSettings().setMotionProfileAccelLimit(decNew), PIDConfigSlot.kSlot0);
      dec = decNew;
    }
    double velNew = velEntry.getDouble(vel);
    if (velNew != vel) {
      settings.setPIDSettings(
          new PIDSettings().setMotionProfileAccelLimit(velNew), PIDConfigSlot.kSlot0);
      vel = velNew;
    }
    double setpointNew = setpointEntry.getDouble(setpoint);
    if (setpointNew != setpoint) {
      setpoint = setpointNew;
      newPos = setpoint;
    }
    return newPos;
  }
}
