package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorIOTalonFX implements ElevatorIO {
  private TalonFX leaderMotor;
  private TalonFX followerMotor;
  private double lastRequestedPosMeters;
  private double lastRequestedPosRotations;
  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  public ElevatorIOTalonFX() {
    leaderMotor = new TalonFX(Constants.Elevator.frontMotorID);
    followerMotor = new TalonFX(Constants.Elevator.backMotorID);

    // Setup config objects
    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.Elevator.statorCurrentLimitAmps;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.supplyCurrentLimitAmps;

    motorConfigs.MotorOutput.Inverted = Constants.Elevator.motorInversion;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfigs.Slot0.kD = Constants.Elevator.fast_kD;
    motorConfigs.Slot0.kP = Constants.Elevator.fast_kP;
    motorConfigs.Slot0.kI = Constants.Elevator.fast_kI;
    motorConfigs.Slot0.kG = Constants.Elevator.kG;
    motorConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    motorConfigs.Slot1.kD = Constants.Elevator.slow_kD;
    motorConfigs.Slot1.kP = Constants.Elevator.slow_kP;
    motorConfigs.Slot1.kI = Constants.Elevator.slow_kI;
    motorConfigs.Slot1.kG = Constants.Elevator.kG;
    motorConfigs.Slot1.GravityType = GravityTypeValue.Elevator_Static;

    motorConfigs.MotionMagic.MotionMagicAcceleration =
        (Constants.Elevator.fastAccelerationMetersPerSec2
                / (Math.PI * Constants.Elevator.beltPulleyPitchDiameterMeters))
            * Constants.Elevator.gearRatio;
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity =
        (Constants.Elevator.fastDecelerationMetersPerSec2
                / (Math.PI * Constants.Elevator.beltPulleyPitchDiameterMeters))
            * Constants.Elevator.gearRatio;
    motorConfigs.MotionMagic.MotionMagicJerk = Constants.Elevator.motionMagicJerk;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    motorConfigs.Feedback.SensorToMechanismRatio = Constants.Elevator.gearRatio;

    StatusCode leaderConfigStatus = leaderMotor.getConfigurator().apply(motorConfigs);
    StatusCode followerConfigStatus = followerMotor.getConfigurator().apply(motorConfigs);
    StatusCode followerModeSetStatus =
        followerMotor.setControl(new Follower(Constants.Elevator.backMotorID, true));

    if (leaderConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + leaderMotor.getDeviceID()
              + " error (Right Elevator): "
              + leaderConfigStatus.getDescription(),
          false);
    }

    if (followerConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + followerMotor.getDeviceID()
              + " error (Left Elevator): "
              + followerConfigStatus.getDescription(),
          false);
    }

    if (followerModeSetStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + followerMotor.getDeviceID()
              + " error (setting follower mode): "
              + followerModeSetStatus.getDescription(),
          false);
    }
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Implementation for updating inputs from the TalonFX hardware
    inputs.leaderConnected = leaderMotor.isConnected();
    inputs.followerConnected = followerMotor.isConnected();

    inputs.requestedPosMeters = lastRequestedPosMeters;
    inputs.requestedPosRotations = lastRequestedPosRotations;

    inputs.leaderheightMeters = rotationsToMeters(leaderMotor.getPosition().getValueAsDouble());
    inputs.followerHeightMeters = rotationsToMeters(followerMotor.getPosition().getValueAsDouble());

    inputs.followerVelocityMetersPerSecond = followerMotor.getVelocity().getValueAsDouble();
    inputs.velMetersPerSecond = rotationsToMeters(leaderMotor.getVelocity().getValueAsDouble());

    // inputs.leaderSupplyAmps = leaderMotor.getBusCurrent().getValueAsDouble();
    // inputs.followerSupplyAmps = followerMotor.getBusCurrent().getValueAsDouble();

    inputs.leaderStatorAmps = leaderMotor.getStatorCurrent().getValueAsDouble();
    inputs.followerStatorAmps = followerMotor.getStatorCurrent().getValueAsDouble();

    inputs.leadertempCelcius = leaderMotor.getDeviceTemp().getValueAsDouble();
    inputs.followertempCelcius = followerMotor.getDeviceTemp().getValueAsDouble();

    // inputs.leaderControllerTempCelcius = leaderMotor.getControllerTemperatureFrame().getValue();
    // inputs.followerControllerTempCelcius =
    // followerMotor.getControllerTemperatureFrame().getValue();

    inputs.leaderVoltage = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.leaderEncoderRotations = leaderMotor.getPosition().getValueAsDouble();

    inputs.followerVoltage = followerMotor.getMotorVoltage().getValueAsDouble();
    inputs.followerEncoderRotations = followerMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setPosition(double elevatorPositionMeters) {
    lastRequestedPosRotations = metersToRotations(elevatorPositionMeters);
    leaderMotor.setPosition(lastRequestedPosRotations);
    followerMotor.setPosition(lastRequestedPosRotations);
    stop();
  }

  @Override
  public void requestSlowHeightMeters(double heightMeters) {

    lastRequestedPosMeters = heightMeters;
    lastRequestedPosRotations = metersToRotations(heightMeters);
    leaderMotor.setControl(new MotionMagicVoltage(lastRequestedPosRotations));
  }

  @Override
  public void requestHeightMeters(double heightMeters) {

    lastRequestedPosMeters = heightMeters;
    lastRequestedPosRotations = metersToRotations(heightMeters);
    leaderMotor.setControl(
        new MotionMagicVoltage(lastRequestedPosRotations).withSlot(0).withEnableFOC(true));
  }

  @Override
  public void setVoltage(double voltage) {
    leaderMotor.setVoltage(voltage);
    lastRequestedPosMeters = -1;
    lastRequestedPosRotations = -1;
  }

  @Override
  public void stop() {
    leaderMotor.stopMotor();
    followerMotor.stopMotor();
    lastRequestedPosMeters = -1;
    lastRequestedPosRotations = -1;
  }

  @Override
  public TalonFX getTalonFX() {
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

  @Override
  public void enableBrakeMode(boolean enable) {
    leaderMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    followerMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
