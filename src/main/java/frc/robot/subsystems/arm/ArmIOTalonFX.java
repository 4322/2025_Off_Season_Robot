package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIOTalonFX implements ArmIO {
  private TalonFX armMotor;
  private double lastRequestedPosDeg;

  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private double setpointMechanismRot;

  public ArmIOTalonFX() {
    armMotor = new TalonFX(Constants.Arm.armMotorId);

    StatusCode armConfigStatus = configarm();

    if (armConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon" + armMotor.getDeviceID() + " error (Arm): " + armConfigStatus.getDescription(),
          false);
    }
  }

  private StatusCode configarm() {
    motorConfig.CurrentLimits.StatorCurrentLimit = Constants.Arm.statorCurrentLimitAmps;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Arm.supplyCurrentLimitAmps;

    motorConfig.MotorOutput.Inverted = Constants.Arm.motorInversion;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.Slot0.kP = Constants.Arm.kP;
    motorConfig.Slot0.kD = Constants.Arm.kD;
    motorConfig.Slot0.kI = Constants.Arm.kI;

    motorConfig.Slot1.kP = Constants.Arm.kP;
    motorConfig.Slot1.kD = Constants.Arm.kD;
    motorConfig.Slot1.kI = Constants.Arm.kI;

    motorConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    motorConfig.Feedback.SensorToMechanismRatio = Constants.Arm.gearRatio;

    motorConfig.MotionMagic.MotionMagicAcceleration =
        Constants.Arm.accelerationLimitCoral / (Math.PI * Constants.Arm.gearRatio);
    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        Constants.Arm.deaccelerationLimitCoral / (Math.PI * Constants.Arm.gearRatio);

    motorConfig.MotionMagic.MotionMagicJerk = Constants.Arm.motionMagicJerk;

    motorConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    return armMotor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.requestedPosDeg = lastRequestedPosDeg;
    inputs.PositionDegrees =
        Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble())
            - Constants.Arm.OffsetEncoderDeg;
    inputs.armConnected = armMotor.isConnected();
    inputs.voltage = armMotor.getMotorVoltage().getValueAsDouble();
    inputs.velocityDegSec = Units.rotationsToDegrees(armMotor.getVelocity().getValueAsDouble());
    inputs.SupplyCurrentAmps = armMotor.getSupplyCurrent().getValueAsDouble();
    inputs.StatorCurrentAmps = armMotor.getStatorCurrent().getValueAsDouble();
    inputs.motorTempCelsius = armMotor.getDeviceTemp().getValueAsDouble();
    inputs.tempCelcius =
        new double[] {
          armMotor.getDeviceTemp().getValueAsDouble(),
        };
  }

  @Override
  public void setHomePosition(double degrees) {
    armMotor.setPosition(degrees);
    stopArmMotor();
  }

  @Override
  public void requestPositionCoral(double requestSetpoint) {
    armMotor.setControl(
        new MotionMagicVoltage(
                Units.degreesToRotations(requestSetpoint + Constants.Arm.OffsetEncoderDeg))
            .withSlot(0)
            .withEnableFOC(true));
    lastRequestedPosDeg = requestSetpoint;
  }

  @Override
  public void requestPositionAlgae(double requestSetpoint) {
    armMotor.setControl(
        new MotionMagicVoltage(
                Units.degreesToRotations(requestSetpoint + Constants.Arm.OffsetEncoderDeg))
            .withSlot(1)
            .withEnableFOC(true));
    lastRequestedPosDeg = requestSetpoint;
  }

  @Override
  public void setVoltage(double voltage) {
    armMotor.setControl(new VoltageOut(voltage));
    lastRequestedPosDeg = -1;
  }

  @Override
  public void stopArmMotor() {
    armMotor.stopMotor();
    lastRequestedPosDeg = -1;
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    armMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
