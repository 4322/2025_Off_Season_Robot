package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.BabyAlchemist;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import frc.robot.util.SwerveUtil.SwerveModuleConstants;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants constants;
  private final Timer initTimer = new Timer();

  public Module(ModuleIO io, int index, SwerveModuleConstants constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
    initTimer.start();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    if (Constants.driveMode == SubsystemMode.TUNING && index == 0 && initTimer.hasElapsed(5)) {
      Double newPos = BabyAlchemist.run(io.getTurnNitrate());
      if (newPos != null) {
        io.setTurnPosition(new Rotation2d(Units.degreesToRadians(newPos)));
      }
    }
  }

  public void runClosedLoopDrive(SwerveModuleState state) {
    if (Constants.driveMode == SubsystemMode.NORMAL) {
      // Optimize velocity setpoint
      state.optimize(getAngle());
      state.cosineScale(inputs.turnPosition);

      // Apply setpoints
      io.setDriveVelocity(state.speedMetersPerSecond / constants.driveWheelRadius);
      io.setTurnPosition(state.angle);
    }
  }

  public void runOpenLoopDrive(SwerveModuleState state) {
    if (Constants.driveMode == SubsystemMode.NORMAL) {
      // Optimize velocity setpoint
      state.optimize(getAngle());
      state.cosineScale(getAngle());

      // Apply setpoints
      io.setDriveOpenLoop(state.speedMetersPerSecond / constants.driveWheelRadius);
      io.setTurnPosition(state.angle);
    }
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.driveWheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.driveWheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }
}
