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
  private double lastRequestedPosMeters;
  PIDPositionRequest elvSlowPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);
  PIDPositionRequest elvPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot1, 0).useMotionProfile(true);

  public ElevatorIONitrate() {
    // Initialize leader and follower motors
    leaderMotor = new Nitrate(Constants.Elevator.frontMotorID, MotorType.kCu60);
    followerMotor = new Nitrate(Constants.Elevator.backMotorID, MotorType.kCu60);

    // Setup config objects
    NitrateSettings frontElevatorMotorConfig = new NitrateSettings();
    OutputSettings frontElevatorMotorOutputSettings = new OutputSettings();
    PIDSettings elevatorPIDSettings = new PIDSettings();
    PIDSettings elevatorSlowPIDSettings = new PIDSettings();
    ElectricalLimitSettings elevatorElectricalLimitSettings = new ElectricalLimitSettings();
    FollowMotorRequest followerRequest = new FollowMotorRequest(leaderMotor);

    elevatorPIDSettings.setPID(
        Constants.Elevator.fast_kP, Constants.Elevator.fast_kI, Constants.Elevator.fast_kD);
    elevatorPIDSettings.setGravitationalFeedforward(Constants.Elevator.kG);
    elevatorPIDSettings.setMinwrapConfig(new MinwrapConfig.Disabled());
    elevatorPIDSettings.setMotionProfileAccelLimit(
        metersToRotations(Constants.Elevator.fastAccelerationMetersPerSec2));
    elevatorPIDSettings.setMotionProfileDeaccelLimit(
        metersToRotations(Constants.Elevator.fastDecelerationMetersPerSec2));
    elevatorPIDSettings.setMotionProfileVelocityLimit(
        metersToRotations(Constants.Elevator.fastVelocityMetersPerSec));

    elevatorSlowPIDSettings.setPID(
        Constants.Elevator.slow_kP, Constants.Elevator.slow_kI, Constants.Elevator.slow_kD);
    elevatorSlowPIDSettings.setGravitationalFeedforward(Constants.Elevator.kG);
    elevatorSlowPIDSettings.setMinwrapConfig(new MinwrapConfig.Disabled());
    elevatorSlowPIDSettings.setMotionProfileAccelLimit(
        metersToRotations(Constants.Elevator.slowAccelerationMetersPerSec2));
    elevatorSlowPIDSettings.setMotionProfileDeaccelLimit(
        metersToRotations(Constants.Elevator.slowDecelerationMetersPerSec2));
    elevatorSlowPIDSettings.setMotionProfileVelocityLimit(
        metersToRotations(Constants.Elevator.slowVelocityMetersPerSec));

    frontElevatorMotorOutputSettings.setIdleMode(Constants.Elevator.motorIdleMode);
    frontElevatorMotorOutputSettings.setInvert(Constants.Elevator.motorFrontInvert);

    elevatorElectricalLimitSettings.setBusCurrentLimit(Constants.Elevator.supplyCurrentLimitAmps);
    elevatorElectricalLimitSettings.setStatorCurrentLimit(
        Constants.Elevator.statorCurrentLimitAmps);

    followerRequest.setInverted(true);

    frontElevatorMotorConfig.setElectricalLimitSettings(elevatorElectricalLimitSettings);
    frontElevatorMotorConfig.setPIDSettings(elevatorPIDSettings, PIDConfigSlot.kSlot0);
    frontElevatorMotorConfig.setPIDSettings(elevatorSlowPIDSettings, PIDConfigSlot.kSlot1);
    frontElevatorMotorConfig.setOutputSettings(frontElevatorMotorOutputSettings);

    NitrateSettings leaderConfigStatus = leaderMotor.setSettings(frontElevatorMotorConfig, 0.02, 5);
    NitrateSettings followerConfigStatus =
        followerMotor.setSettings(frontElevatorMotorConfig, 0.02, 5);
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
    inputs.leaderElevatorMotorConnected = leaderMotor.isConnected();
    inputs.followerElevatorMotorConnected = followerMotor.isConnected();

    inputs.requestedPosMeters = lastRequestedPosMeters;

    inputs.leaderMotorheightMeters = rotationsToMeters(leaderMotor.getPosition());
    inputs.followerMotorheightMeters = rotationsToMeters(followerMotor.getPosition());

    inputs.leaderMotorVoltage = leaderMotor.getBusVoltageFrame().getValue();
    inputs.followerMotorVoltage = followerMotor.getBusVoltageFrame().getValue();

    inputs.followerMotorVelocityMetersPerSecond = followerMotor.getVelocity();
    inputs.leaderMotorVelocityMetersPerSecond = leaderMotor.getVelocity();

    inputs.leaderMotorSupplyCurrentAmps = leaderMotor.getBusCurrent();
    inputs.followerMotorSupplyCurrentAmps = followerMotor.getBusCurrent();

    inputs.leaderMotorStatorCurrentAmps = leaderMotor.getStatorCurrent();
    inputs.followerMotorStatorCurrentAmps = followerMotor.getStatorCurrent();

    inputs.leaderMotortempCelcius = leaderMotor.getMotorTemperatureFrame().getValue();
    inputs.followerMotortempCelcius = followerMotor.getMotorTemperatureFrame().getValue();
  }

  @Override
  public void setPosition(double elevatorPositionMeters) {
    stop(IdleMode.kBrake);
    leaderMotor.setPosition(metersToRotations(elevatorPositionMeters));
  }

  @Override
  public void requestSlowHeightMeters(double heightMeters) {
    leaderMotor.setRequest(elvSlowPositionRequest.setPosition(metersToRotations(heightMeters)));
    lastRequestedPosMeters = heightMeters;
  }

  @Override
  public void requestHeightMeters(double heightMeters) {
    leaderMotor.setRequest(elvPositionRequest.setPosition(metersToRotations(heightMeters)));
    lastRequestedPosMeters = heightMeters;
  }

  @Override
  public void setVoltage(double voltage) {
    leaderMotor.setVoltage(voltage);
    lastRequestedPosMeters = -1;
  }

  @Override
  public void stop(IdleMode idleMode) {
    leaderMotor.stop(idleMode);
    lastRequestedPosMeters = -1;
  }

  public double metersToRotations(double meters) {
    return (meters / (2 * Math.PI)) * Constants.Elevator.gearRatio;
  }

  public double rotationsToMeters(double rotations) {
    return (rotations * (2 * Math.PI)) / Constants.Elevator.gearRatio;
  }
}
