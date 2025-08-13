package frc.robot.subsystems.elevator;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.OutputSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.MinwrapConfig;
import com.reduxrobotics.motorcontrol.nitrate.types.MotionProfileMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import com.reduxrobotics.motorcontrol.requests.PIDPositionRequest;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class ElevatorIONitrate implements ElevatorIO {
  private final Nitrate leaderMotor;
  private final Nitrate followerMotor;

  private final PIDPositionRequest elevatorPIDPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);

  public ElevatorIONitrate() {
    leaderMotor = new Nitrate(Constants.Elevator.leftMotorID, MotorType.kCu60);
    followerMotor = new Nitrate(Constants.Elevator.rightMotorID, MotorType.kCu60);
    NitrateSettings elevatorConfig = new NitrateSettings();
    OutputSettings elevatorMotorOutputSettings = new OutputSettings();
    elevatorMotorOutputSettings.setIdleMode(Constants.Elevator.motorIdleMode);
    elevatorMotorOutputSettings.setInvert(Constants.Elevator.motorInvert);
    elevatorConfig.setElectricalLimitSettings(Constants.Elevator.elevatorElectricalLimitSettings);
    elevatorConfig
        .setPIDSettings(Constants.Elevator.elevatorMotorGains, PIDConfigSlot.kSlot1)
        .getPIDSettings(PIDConfigSlot.kSlot0)
        .setMotionProfileMode(MotionProfileMode.kTrapezoidal)
        .setMinwrapConfig(new MinwrapConfig.Enabled());
    NitrateSettings leaderConfigStatus = leaderMotor.setSettings(elevatorConfig, 0.02, 5);
    NitrateSettings followerConfigStatus = followerMotor.setSettings(elevatorConfig, 0.02, 5);

    if (!leaderConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + leaderMotor.getAddress().getDeviceId()
              + " (leader motor) failed to configure",
          false);
    }
    if (!followerConfigStatus.isEmpty()) {
      DriverStation.reportError(
          "Nitrate "
              + followerMotor.getAddress().getDeviceId()
              + " (follower motor) failed to configure",
          false);
    }
  }

  public void updateInputs(ElevatorIOInputs elevatorInputs) {
    // Implementation for updating inputs from the Nitrate hardware
  }
}
