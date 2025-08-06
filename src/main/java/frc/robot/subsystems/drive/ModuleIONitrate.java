package frc.robot.subsystems.drive;

import com.reduxrobotics.motorcontrol.nitrate.Nitrate;
import com.reduxrobotics.motorcontrol.nitrate.NitrateSettings;
import com.reduxrobotics.motorcontrol.nitrate.settings.FeedbackSensorSettings;
import com.reduxrobotics.motorcontrol.nitrate.types.AtomicBondMode;
import com.reduxrobotics.motorcontrol.nitrate.types.FeedbackSensor;
import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import com.reduxrobotics.motorcontrol.nitrate.types.InvertMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MinwrapConfig;
import com.reduxrobotics.motorcontrol.nitrate.types.MotionProfileMode;
import com.reduxrobotics.motorcontrol.nitrate.types.MotorType;
import com.reduxrobotics.motorcontrol.nitrate.types.PIDConfigSlot;
import com.reduxrobotics.motorcontrol.requests.PIDPositionRequest;
import com.reduxrobotics.motorcontrol.requests.PIDVelocityRequest;
import com.reduxrobotics.motorcontrol.requests.VoltageRequest;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import edu.wpi.first.math.util.Units;

public class ModuleIONitrate implements ModuleIO {
  private final Nitrate driveMotor;
  private final Nitrate turnMotor;
  private final Canandmag turnEncoder;

  // TODO: Create Swerve Module Constant Factory for Motor + sensor configuration
  public ModuleIONitrate() {
    driveMotor = new Nitrate(0, MotorType.kCu60);
    turnMotor = new Nitrate(0, MotorType.kCu60);
    turnEncoder = new Canandmag(0);

    NitrateSettings driveConfig = new NitrateSettings();
    driveConfig
        .getAtomicBondSettings()
        .setAtomicBondMode(AtomicBondMode.kSwerveModule)
        .setAtomicSwerveConstants(turnMotor, turnEncoder, null, 0, 0);
    driveConfig.getOutputSettings().setIdleMode(IdleMode.kBrake).setInvert(InvertMode.kNotInverted);
    driveConfig
        .getElectricalLimitSettings()
        .setBusCurrentLimit(0)
        .setBusCurrentLimitTime(0)
        .setStatorCurrentLimit(0);
    driveConfig
        .getPIDSettings(PIDConfigSlot.kSlot0)
        .setPID(0, 0, 0)
        .setStaticFeedforward(0)
        .setVelocityFeedforward(0);
    driveMotor.setSettings(driveConfig, 0, 1);

    NitrateSettings turnConfig = new NitrateSettings();
    turnConfig.getAtomicBondSettings()
        .setAtomicBondMode(AtomicBondMode.kSwerveModule)
        .setAtomicSwerveConstants(turnMotor, turnEncoder, null, 0, 0);
    turnConfig.getOutputSettings().setIdleMode(IdleMode.kBrake).setInvert(InvertMode.kInverted);
    turnConfig
        .getFeedbackSensorSettings()
        .setFeedbackSensor(new FeedbackSensor.CanandmagAbsolute(0, 0));
    turnConfig
        .getElectricalLimitSettings()
        .setBusCurrentLimit(0)
        .setBusCurrentLimitTime(0)
        .setStatorCurrentLimit(0);
    turnConfig
        .getPIDSettings(PIDConfigSlot.kSlot0)
        .setPID(0, 0, 0)
        .setStaticFeedforward(0)
        .setVelocityFeedforward(0)
        .setMotionProfileMode(MotionProfileMode.kTrapezoidal)
        .setMotionProfileAccelLimit(0)
        .setMotionProfileDeaccelLimit(0)
        .setMotionProfileVelocityLimit(0)
        .setMinwrapConfig(new MinwrapConfig.Enabled());
    turnMotor.setSettings(turnConfig, 0, 1);

    CanandmagSettings settings = new CanandmagSettings();
    settings.setInvertDirection(false);
    turnEncoder.setSettings(settings, 0, 1);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = driveMotor.isConnected();
    inputs.drivePositionRad = Units.rotationsToRadians(driveMotor.getPosition());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveMotor.getVelocity());
    inputs.driveAppliedVolts = driveMotor.getBusVoltageFrame().getData();
    inputs.driveSupplyCurrentAmps = driveMotor.getBusCurrent();
    inputs.driveStatorCurrentAmps = driveMotor.getStatorCurrent();
    inputs.driveTempCelsius = driveMotor.getMotorTemperatureFrame().getData();

    inputs.turnConnected = turnMotor.isConnected();
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnMotor.getVelocity());
    inputs.turnAppliedVolts = driveMotor.getBusVoltageFrame().getData();
    inputs.turnSupplyCurrentAmps = turnMotor.getBusCurrent();
    inputs.turnStatorCurrentAmps = turnMotor.getStatorCurrent();
    inputs.turnTempCelsius = turnMotor.getMotorTemperatureFrame().getData();

    inputs.turnEncoderConnected = turnEncoder.isConnected();
    inputs.turnAbsolutePositionRad = Units.rotationsToRadians(turnEncoder.getAbsPosition());
    inputs.turnPositionRad = Units.rotationsToRadians(turnEncoder.getPosition());
  }

  @Override
  public void setDriveOpenLoop(double outputVoltage) {
    driveMotor.setRequest(new VoltageRequest(outputVoltage).setVoltageCompensate(true));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveMotor.setRequest(new PIDVelocityRequest(PIDConfigSlot.kSlot0, velocityRadPerSec));
  }

  @Override
  public void setTurnPosition(double positionRad) {
    turnMotor.setRequest(
        new PIDPositionRequest(PIDConfigSlot.kSlot0, positionRad).useMotionProfile(true));
  }
}
