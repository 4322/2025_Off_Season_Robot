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
  private PIDPositionRequest elvSlowPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);
  private PIDPositionRequest elvPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot1, 0).useMotionProfile(true);
  private FollowMotorRequest followerRequest;

  public ElevatorIONitrate() {
    // Initialize leader and follower motors
    leaderMotor = new Nitrate(Constants.Elevator.frontMotorID, MotorType.kCu60);
    followerMotor = new Nitrate(Constants.Elevator.backMotorID, MotorType.kCu60);

    // Setup config objects
    NitrateSettings frontConfig = new NitrateSettings();

    followerRequest = new FollowMotorRequest(leaderMotor);

    frontConfig.setPIDSettings(
        new PIDSettings()
            .setPID(
                Constants.Elevator.fast_kP, Constants.Elevator.fast_kI, Constants.Elevator.fast_kD)
            .setGravitationalFeedforward(Constants.Elevator.kG)
            .setMinwrapConfig(new MinwrapConfig.Disabled())
            .setMotionProfileAccelLimit(
                metersToRotations(Constants.Elevator.fastAccelerationMetersPerSec2))
            .setMotionProfileDeaccelLimit(
                metersToRotations(Constants.Elevator.fastDecelerationMetersPerSec2))
            .setMotionProfileVelocityLimit(
                metersToRotations(Constants.Elevator.fastVelocityMetersPerSec))
            .setISaturation(Constants.Elevator.errorUnit)
            .setIZone(Constants.Elevator.finalOutputDeg),
        PIDConfigSlot.kSlot0);

    frontConfig.setPIDSettings(
        new PIDSettings()
            .setPID(
                Constants.Elevator.slow_kP, Constants.Elevator.slow_kI, Constants.Elevator.slow_kD)
            .setGravitationalFeedforward(Constants.Elevator.kG)
            .setMinwrapConfig(new MinwrapConfig.Disabled())
            .setMotionProfileAccelLimit(
                metersToRotations(Constants.Elevator.slowAccelerationMetersPerSec2))
            .setMotionProfileDeaccelLimit(
                metersToRotations(Constants.Elevator.slowDecelerationMetersPerSec2))
            .setMotionProfileVelocityLimit(
                metersToRotations(Constants.Elevator.slowVelocityMetersPerSec))
            .setISaturation(Constants.Elevator.errorUnit)
            .setIZone(Constants.Elevator.finalOutputDeg),
        PIDConfigSlot.kSlot1);

    frontConfig.setOutputSettings(
        new OutputSettings()
            .setIdleMode(Constants.Elevator.motorIdleMode)
            .setInvert(Constants.Elevator.motorFrontInvert));

    frontConfig.setElectricalLimitSettings(
        new ElectricalLimitSettings()
            .setBusCurrentLimit(Constants.Elevator.supplyCurrentLimitAmps)
            .setStatorCurrentLimit(Constants.Elevator.statorCurrentLimitAmps));

    followerRequest.setInverted(true);

    NitrateSettings leaderConfigStatus = leaderMotor.setSettings(frontConfig, 0.02, 5);
    NitrateSettings followerConfigStatus = followerMotor.setSettings(frontConfig, 0.02, 5);
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
    inputs.leaderConnected = leaderMotor.isConnected();
    inputs.followerConnected = followerMotor.isConnected();

    inputs.requestedPosMeters = lastRequestedPosMeters;

    inputs.leaderheightMeters = rotationsToMeters(leaderMotor.getPosition());
    inputs.followerHeightMeters = rotationsToMeters(followerMotor.getPosition());

    inputs.leaderVoltage = leaderMotor.getBusVoltageFrame().getValue();
    inputs.followerVoltage = followerMotor.getBusVoltageFrame().getValue();

    inputs.followerVelocityMetersPerSecond = followerMotor.getVelocity();
    inputs.leaderVelocityMetersPerSecond = leaderMotor.getVelocity();

    inputs.leaderSupplyAmps = leaderMotor.getBusCurrent();
    inputs.followerSupplyAmps = followerMotor.getBusCurrent();

    inputs.leaderStatorAmps = leaderMotor.getStatorCurrent();
    inputs.followerStatorAmps = followerMotor.getStatorCurrent();

    inputs.leadertempCelcius = leaderMotor.getMotorTemperatureFrame().getValue();
    inputs.followertempCelcius = followerMotor.getMotorTemperatureFrame().getValue();

    inputs.leaderVoltage = leaderMotor.getAppliedVoltageFrame().getValue();
    inputs.leaderEncoderRotations = leaderMotor.getPosition();

    inputs.followerVoltage = followerMotor.getAppliedVoltageFrame().getValue();
    inputs.followerEncoderRotations = followerMotor.getPosition();
  }

  @Override
  public void setPosition(double elevatorPositionMeters) {
    stop(IdleMode.kBrake);
    leaderMotor.setPosition(metersToRotations(elevatorPositionMeters));
    followerMotor.setPosition(metersToRotations(elevatorPositionMeters));
  }

  @Override
  public void requestSlowHeightMeters(double heightMeters) {
    followerMotor.setRequest(followerRequest); // temporary work-around for firmware issue
    leaderMotor.setRequest(elvSlowPositionRequest.setPosition(metersToRotations(heightMeters)));
    lastRequestedPosMeters = heightMeters;
  }

  @Override
  public void requestHeightMeters(double heightMeters) {
    followerMotor.setRequest(followerRequest); // temporary work-around for firmware issue
    leaderMotor.setRequest(elvPositionRequest.setPosition(metersToRotations(heightMeters)));
    lastRequestedPosMeters = heightMeters;
  }

  @Override
  public void setVoltage(double voltage) {
    followerMotor.setRequest(followerRequest); // temporary work-around for firmware issue
    leaderMotor.setVoltage(voltage);
    lastRequestedPosMeters = -1;
  }

  @Override
  public void stop(IdleMode idleMode) {
    followerMotor.setRequest(followerRequest); // temporary work-around for firmware issue
    leaderMotor.stop(idleMode);
    lastRequestedPosMeters = -1;
  }

  @Override
  public Nitrate getNitrate() {
    followerMotor.setRequest(followerRequest); // temporary work-around for firmware issue
    return leaderMotor;
  }

  public double metersToRotations(double meters) {
    return meters
        / (Math.PI * Constants.Elevator.beltPulleyPitchDiameterMeters)
        * Constants.Elevator.gearRatio;
  }

  public double rotationsToMeters(double rotations) {
    return rotations
        * Math.PI
        * Constants.Elevator.beltPulleyPitchDiameterMeters
        / Constants.Elevator.gearRatio;
  }
}
