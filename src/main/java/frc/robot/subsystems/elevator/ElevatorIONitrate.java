package frc.robot.subsystems.elevator;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.ElectricalLimitSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.OutputSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.PIDSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MinwrapConfig;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import com.reduxrobotics.motorcontrol.requests.FollowMotorRequest;
import com.reduxrobotics.motorcontrol.requests.PIDPositionRequest;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class ElevatorIONitrate implements ElevatorIO {
  private final Nitrate leaderMotor;
  private final Nitrate followerMotor;
  PIDPositionRequest elvPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);
  // private final PIDPositionRequest elevatorPIDPositionRequest =
  // new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);
  // private final PIDPositionRequest elevatorPIDPositionRequest =
  // new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);

  public ElevatorIONitrate() {
    // Initialize leader and follower motors
    leaderMotor = new Nitrate(Constants.Elevator.leftMotorID, MotorType.kCu60);
    followerMotor = new Nitrate(Constants.Elevator.rightMotorID, MotorType.kCu60);
    // setup objects
    NitrateSettings elevatorConfig = new NitrateSettings();
    OutputSettings elevatorMotorOutputSettings = new OutputSettings();

    PIDSettings elevatorPIDSettings = new PIDSettings();
    elevatorPIDSettings.setPID(
        Constants.Elevator.kP0, Constants.Elevator.kI0, Constants.Elevator.kD0);
    elevatorPIDSettings.setGravitationalFeedforward(Constants.Elevator.kG);
    elevatorPIDSettings.setMinwrapConfig(new MinwrapConfig.Disabled());
    elevatorPIDSettings.setMotionProfileAccelLimit(Constants.Elevator.AccelerationLimit);
    elevatorPIDSettings.setMotionProfileDeaccelLimit(Constants.Elevator.DeaccelerationLimit);
    elevatorPIDSettings.setMotionProfileVelocityLimit(Constants.Elevator.VelocityLimit);

    PIDSettings ElevatorSlowPIDSettings = new PIDSettings();
    ElevatorSlowPIDSettings.setPID(
        Constants.Elevator.kP1, Constants.Elevator.kI1, Constants.Elevator.kD1);
    ElevatorSlowPIDSettings.setGravitationalFeedforward(Constants.Elevator.kG);
    ElevatorSlowPIDSettings.setMinwrapConfig(new MinwrapConfig.Disabled());
    ElevatorSlowPIDSettings.setMotionProfileAccelLimit(Constants.Elevator.AccelerationLimit);
    ElevatorSlowPIDSettings.setMotionProfileDeaccelLimit(Constants.Elevator.DeaccelerationLimit);
    ElevatorSlowPIDSettings.setMotionProfileVelocityLimit(Constants.Elevator.VelocityLimit);

    PIDSettings elevatorMotorPIDSettings = new PIDSettings();
    ElectricalLimitSettings elevatorElectricalLimitSettings = new ElectricalLimitSettings();
    FollowMotorRequest followerRequest = new FollowMotorRequest(leaderMotor);
    // configs
    elevatorMotorOutputSettings.setIdleMode(Constants.Elevator.motorIdleMode);
    elevatorMotorOutputSettings.setInvert(Constants.Elevator.motorInvert); // make this leader motor
    // invert follower
    elevatorConfig.setOutputSettings(elevatorMotorOutputSettings);
    elevatorElectricalLimitSettings.setBusCurrentLimit(Constants.Elevator.supplyCurrentLimitAmps);
    elevatorElectricalLimitSettings.setStatorCurrentLimit(
        Constants.Elevator.statorCurrentLimitAmps);
    elevatorConfig.setElectricalLimitSettings(elevatorElectricalLimitSettings);
    elevatorConfig.setPIDSettings(elevatorMotorPIDSettings, PIDConfigSlot.kSlot0);
    elevatorConfig.setPIDSettings(ElevatorSlowPIDSettings, PIDConfigSlot.kSlot1);
    NitrateSettings leaderConfigStatus = leaderMotor.setSettings(elevatorConfig, 0.02, 5);
    NitrateSettings followerConfigStatus = followerMotor.setSettings(elevatorConfig, 0.02, 5);
    followerMotor.setRequest(followerRequest);
    // get position is an internal encoder, so we need to set it
    // 6 to 1 gear ratio for elevator first stage
    // 9 to 1 gear ratio for elevator second stage

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

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Implementation for updating inputs from the Nitrate hardware
    inputs.elevatorMotorConnected = leaderMotor.isConnected();
    inputs.heightMeters = leaderMotor.getPosition();
    inputs.velocityMetersSecond = leaderMotor.getVelocity();
    inputs.supplyCurrentAmps = leaderMotor.getBusCurrent();
    inputs.statorCurrentAmps = leaderMotor.getStatorCurrent();
    inputs.tempCelcius = leaderMotor.getMotorTemperatureFrame().getValue();
  }

  @Override
  public void setPosition(double elevatorPositionMeters) {
    leaderMotor.setPosition(metersToRotations(elevatorPositionMeters));
    followerMotor.setPosition(metersToRotations(elevatorPositionMeters));
  }

  @Override
  public void requestHeight(double heightMeters) {
    leaderMotor.setRequest(elvPositionRequest.setPosition(metersToRotations(heightMeters)));
  }

  @Override
  public void setVoltage(double voltage) {
    leaderMotor.setVoltage(voltage);
    followerMotor.setVoltage(voltage);
  }

  @Override
  public void setNeutralMode(IdleMode idleMode) {
    leaderMotor.stop(idleMode);
    followerMotor.stop(idleMode);
  }

  public double metersToRotations(double meters) {
    return (meters / (2 * Math.PI)) * Constants.Elevator.gearRatio;
  }
}
