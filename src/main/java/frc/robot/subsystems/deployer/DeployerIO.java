package frc.robot.subsystems.deployer;

import com.reduxrobotics.motorcontrol.nitrate.types.IdleMode;
import org.littletonrobotics.junction.AutoLog;

public interface DeployerIO {

  @AutoLog
  public static class DeployerIOInputs {

    public boolean deployerMotorConnected = false;
    public double deployerMotorStatorCurrentAmps = 0.0;
    public double deployerMotorBusCurrentAmps = 0.0;
    public double deployerMotorTempCelcius = 0.0;
    public double deployerMotorSpeedRotationsPerSec = 0.0;
    public double deployerMotorAppliedVolts = 0.0;
    public double deployerMotorPositionRotations = 0.0;

    /* TODO uncomment if external encoder is used
    public boolean deployerMotorEncoderConnected = false;
    public double deployerMotorEncoderSpeedRotationsPerSec = 0.0;
    public double deployerMotorEncoderPositionRotations = 0.0;
    public double deployerMotorEncoderAbsolutePosition = 0.0;
    public double deployerMotorEncoderTempCelcius = 0.0;
    */
  }

  public default void updateInputs(DeployerIOInputs inputs) {}

  public default void setDeployerMotorPosition(double rotations) {}

  public default void stopDeployerMotor(IdleMode idleMode) {}

  public default void deployerMotorEncoderSetHome() {}
}
