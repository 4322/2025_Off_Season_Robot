package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import frc.robot.subsystems.endEffector.EndEffectorIO.EndEffectorIOInputs.gamePiece;

public class EndEffectorIOTalonFX implements EndEffectorIO {
  private TalonFX endEffectorMotor;
  private Canandcolor endEffectorSensor;

  private double previousRequestedVoltage = -999;

  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  public EndEffectorIOTalonFX() {
    endEffectorMotor = new TalonFX(Constants.EndEffector.motorId);

    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.EndEffector.statorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.EndEffector.busCurrentLimit;

    motorConfigs.MotorOutput.Inverted = Constants.EndEffector.motorInvertPhoenix;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode feederConfigStatus = endEffectorMotor.getConfigurator().apply(motorConfigs);

    if (feederConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + endEffectorMotor.getDeviceID()
              + " error (End Effector): "
              + feederConfigStatus.getDescription(),
          false);
    }

    endEffectorSensor = new Canandcolor(Constants.EndEffector.sensorId);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.motorConnected = endEffectorMotor.isConnected();

    inputs.speedRotationsPerSec = endEffectorMotor.getVelocity().getValueAsDouble();
    inputs.statorCurrentAmps = endEffectorMotor.getStatorCurrent().getValueAsDouble();
    inputs.motorTempCelcius = endEffectorMotor.getDeviceTemp().getValueAsDouble();
    inputs.busCurrentAmps = endEffectorMotor.getSupplyCurrent().getValueAsDouble();
    inputs.appliedVolts = endEffectorMotor.getMotorVoltage().getValueAsDouble();

    inputs.sensorConnected = endEffectorSensor.isConnected();
    inputs.sensorProximity = endEffectorSensor.getProximity();
    inputs.sensorColorBlue = endEffectorSensor.getBlue();
    inputs.sensorColorGreen = endEffectorSensor.getGreen();
    inputs.sensorColorRed = endEffectorSensor.getRed();

    inputs.isCoralProximityDetected =
        endEffectorSensor.getProximity() < Constants.EndEffector.coralProximityThreshold;
    inputs.isAlgaeProximityDetected =
        endEffectorSensor.getProximity() < Constants.EndEffector.algaeProximityThreshold;
    // than coral is

    // Enable color detection based on Constant setting
    if (Constants.EndEffector.useSensorColor) {
      if (inputs.isAlgaeProximityDetected) {

        // Green detected is within range; Blue detected is within range; Red detected is below
        // threshold
        if (inputs.sensorColorGreen > Constants.EndEffector.greenDetectGreenLower
            && inputs.sensorColorGreen < Constants.EndEffector.greenDetectGreenUpper
            && inputs.sensorColorBlue > Constants.EndEffector.greenDetectBlueLower
            && inputs.sensorColorBlue < Constants.EndEffector.greenDetectBlueUpper
            && inputs.sensorColorRed < Constants.EndEffector.greenDetectRed) {
          inputs.sensorPieceDetected = gamePiece.ALGAE;

          // All colors detected are above threshold
        } else if (inputs.sensorColorGreen > Constants.EndEffector.whiteDetectGreen
            && inputs.sensorColorBlue > Constants.EndEffector.whiteDetectBlue
            && inputs.sensorColorRed > Constants.EndEffector.whiteDetectRed) {
          inputs.sensorPieceDetected = gamePiece.CORAL;
        } else {
          inputs.sensorPieceDetected = gamePiece.UNKNOWN;
        }
      } else {
        inputs.sensorPieceDetected = gamePiece.NONE;
      }
    } else {
      if (inputs.isAlgaeProximityDetected) {
        inputs.sensorPieceDetected = gamePiece.ALGAE;
      } else if (inputs.isCoralProximityDetected) {
        inputs.sensorPieceDetected = gamePiece.CORAL;
      } else {
        inputs.sensorPieceDetected = gamePiece.NONE;
      }
    }
  }

  @Override
  public void setVoltage(double voltage) {
    if (voltage != previousRequestedVoltage || Constants.continuousNitrateRequestsEnabled) {
      previousRequestedVoltage = voltage;
      endEffectorMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
    }
  }

  @Override
  public void stop() {
    endEffectorMotor.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    endEffectorMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public TalonFX getTalonFX() {
    return endEffectorMotor;
  }
}
