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
  PIDPositionRequest elvSlowPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot0, 0).useMotionProfile(true);
  PIDPositionRequest elvPositionRequest =
      new PIDPositionRequest(PIDConfigSlot.kSlot1, 0).useMotionProfile(true);

  public ElevatorIONitrate() {
    // Initialize leader and follower motors
    leaderMotor = new Nitrate(Constants.Elevator.leftMotorID, MotorType.kCu60);
    followerMotor = new Nitrate(Constants.Elevator.rightMotorID, MotorType.kCu60);
    // setup objects
    NitrateSettings leftElevatorMotorConfig = new NitrateSettings();
    NitrateSettings rightElevatorMotorConfig = new NitrateSettings();

    OutputSettings leftElevatorMotorOutputSettings = new OutputSettings();
    OutputSettings rightElevatorMotorOutputSettings = new OutputSettings();

    PIDSettings elevatorPIDSettings = new PIDSettings();
    elevatorPIDSettings.setPID(
        Constants.Elevator.kP0, Constants.Elevator.kI0, Constants.Elevator.kD0);
    elevatorPIDSettings.setGravitationalFeedforward(Constants.Elevator.kG);
    elevatorPIDSettings.setMinwrapConfig(new MinwrapConfig.Disabled());
    elevatorPIDSettings.setMotionProfileAccelLimit(Constants.Elevator.AccelerationLimit);
    elevatorPIDSettings.setMotionProfileDeaccelLimit(Constants.Elevator.DeaccelerationLimit);
    elevatorPIDSettings.setMotionProfileVelocityLimit(Constants.Elevator.VelocityLimit);

    PIDSettings elevatorSlowPIDSettings = new PIDSettings();
    elevatorSlowPIDSettings.setPID(
        Constants.Elevator.kP1, Constants.Elevator.kI1, Constants.Elevator.kD1);
    elevatorSlowPIDSettings.setGravitationalFeedforward(Constants.Elevator.kG);
    elevatorSlowPIDSettings.setMinwrapConfig(new MinwrapConfig.Disabled());
    elevatorSlowPIDSettings.setMotionProfileAccelLimit(Constants.Elevator.AccelerationLimit);
    elevatorSlowPIDSettings.setMotionProfileDeaccelLimit(Constants.Elevator.DeaccelerationLimit);
    elevatorSlowPIDSettings.setMotionProfileVelocityLimit(Constants.Elevator.VelocityLimit);

    ElectricalLimitSettings elevatorElectricalLimitSettings = new ElectricalLimitSettings();
    FollowMotorRequest followerRequest = new FollowMotorRequest(leaderMotor);
    // configs

    leftElevatorMotorOutputSettings.setIdleMode(Constants.Elevator.motorIdleMode);
    leftElevatorMotorOutputSettings.setInvert(
        Constants.Elevator.motorLeftInvert); // make this leader motor
    // invert follower
    rightElevatorMotorOutputSettings.setIdleMode(Constants.Elevator.motorIdleMode);
    rightElevatorMotorOutputSettings.setInvert(
        Constants.Elevator.motorRightInvert); // make this follower motor

    leftElevatorMotorConfig.setOutputSettings(leftElevatorMotorOutputSettings);
    rightElevatorMotorConfig.setOutputSettings(rightElevatorMotorOutputSettings);

    elevatorElectricalLimitSettings.setBusCurrentLimit(Constants.Elevator.supplyCurrentLimitAmps);
    elevatorElectricalLimitSettings.setStatorCurrentLimit(
        Constants.Elevator.statorCurrentLimitAmps);

    leftElevatorMotorConfig.setElectricalLimitSettings(elevatorElectricalLimitSettings);
    rightElevatorMotorConfig.setElectricalLimitSettings(elevatorElectricalLimitSettings);

    leftElevatorMotorConfig.setPIDSettings(elevatorPIDSettings, PIDConfigSlot.kSlot0);
    leftElevatorMotorConfig.setPIDSettings(elevatorSlowPIDSettings, PIDConfigSlot.kSlot1);
    rightElevatorMotorConfig.setPIDSettings(elevatorPIDSettings, PIDConfigSlot.kSlot0);
    rightElevatorMotorConfig.setPIDSettings(elevatorSlowPIDSettings, PIDConfigSlot.kSlot1);
    NitrateSettings leaderConfigStatus = leaderMotor.setSettings(leftElevatorMotorConfig, 0.02, 5);
    NitrateSettings followerConfigStatus =
        followerMotor.setSettings(rightElevatorMotorConfig, 0.02, 5);
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

    inputs.leaderMotorheightMeters = leaderMotor.getPosition();
    inputs.followerMotorheightMeters = followerMotor.getPosition();
    
    inputs.followerMotorVelocityMetersSecond = followerMotor.getVelocity();
    inputs.leaderMotorVelocityMetersSecond = leaderMotor.getVelocity();

    inputs.leaderMotorSupplyCurrentAmps = leaderMotor.getBusCurrent();
    inputs.followerMotorSupplyCurrentAmps = followerMotor.getBusCurrent();

    inputs.leaderMotorStatorCurrentAmps = leaderMotor.getStatorCurrent();
    inputs.followerMotorStatorCurrentAmps = followerMotor.getStatorCurrent();

    inputs.leaderMotortempCelcius = leaderMotor.getMotorTemperatureFrame().getValue();
    inputs.followerMotortempCelcius = followerMotor.getMotorTemperatureFrame().getValue();
  }

  @Override
  public void setPosition(double elevatorPositionMeters) {
    leaderMotor.setPosition(metersToRotations(elevatorPositionMeters));
  }

  @Override
  public void requestSlowHeightMeters(double heightMeters) {
    leaderMotor.setRequest(elvSlowPositionRequest.setPosition(metersToRotations(heightMeters)));
  }

  @Override
  public void requestHeightMeters(double heightMeters) {
    leaderMotor.setRequest(elvPositionRequest.setPosition(metersToRotations(heightMeters)));
  }

  @Override
  public void setVoltage(double voltage) {
    leaderMotor.setVoltage(voltage);
  }

  @Override
  public void setNeutralMode(IdleMode idleMode) {
    leaderMotor.stop(idleMode);
  }

  public double metersToRotations(double meters) {
    return (meters / (2 * Math.PI)) * Constants.Elevator.gearRatio;
  }
}
