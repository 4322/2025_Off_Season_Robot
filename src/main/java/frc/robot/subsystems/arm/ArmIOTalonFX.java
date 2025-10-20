package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIOTalonFX implements ArmIO {
  private TalonFX armMotor;
  private Canandmag pivotEncoder;

  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private double setpointMechanismRot;

  public ArmIOTalonFX() {
    armMotor = new TalonFX(Constants.Arm.armMotorId);

    StatusCode pivotConfigStatus = configarm();

    if (pivotConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + armMotor.getDeviceID()
              + " error (Algae Pivot): "
              + pivotConfigStatus.getDescription(),
          false);
    }

    // TODO: Set configs for encoder referencing Redux docs
  }

  private StatusCode configarm() {
    motorConfig.CurrentLimits.StatorCurrentLimit = Constants.Arm.statorCurrentLimitAmps;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Arm.supplyCurrentLimitAmps;

    // armConfig.MotorOutput.Inverted = Constants.Arm.motorInversion;
    // armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.Slot0.kP = Constants.Arm.kP;
    motorConfig.Slot0.kD = Constants.Arm.kD;

    motorConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    return armMotor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // inputs.requestedPosDeg = lastRequestedPosDeg;
    // inputs.PositionDegrees =
    //     Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble())
    //         - Constants.Arm.OffsetEncoderDeg;
    inputs.armConnected = armMotor.isConnected();
    // inputs.voltage = armMotor.getBusVoltageFrame().getValueAsDouble();
    // inputs.velocityDegSec = Units.rotationsToDegrees(armMotor.getVelocity().getValueAsDouble());
    inputs.SupplyCurrentAmps = armMotor.getSupplyCurrent().getValueAsDouble();
    inputs.StatorCurrentAmps = armMotor.getStatorCurrent().getValueAsDouble();
    inputs.motorTempCelsius = armMotor.getDeviceTemp().getValueAsDouble();
    // inputs.tempCelcius =
    //     new double[] {
    //       armMotor.getDeviceTemp().getValueAsDouble(),
    //     };
    // inputs.armEncoderConnected = armEncoder.isConnected();
    // // inputs.voltage = armMotor.getAppliedVolts().getValueAsDouble();
    // inputs.encoderArmRotations = armEncoder.getPosition() / Constants.Arm.sensorToArm;
  }
}
