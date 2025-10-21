package frc.robot.subsystems.drivePan;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.BabyAlchemist;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.DrivePanTuningMode;
import frc.robot.constants.Constants.SubsystemMode;
import frc.robot.util.SwerveUtil.SwerveModuleConstants;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants constants;

  public Module(ModuleIO io, int index, SwerveModuleConstants constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("DrivePan/Module" + Integer.toString(index), inputs);

    if (Constants.drivePanMode == SubsystemMode.TUNING) {
      if (Constants.drivePanTuningMode == DrivePanTuningMode.TURNING) {
        Double newPos =
            BabyAlchemist.run(
                index, io.getTurnSalt(), "Turning", inputs.turnPosition.getDegrees(), "degrees");
        if (newPos != null) {
          io.setTurnPosition(new Rotation2d(Units.degreesToRadians(newPos)));
        }
      } else if (Constants.drivePanTuningMode == DrivePanTuningMode.DRIVING_FIXED_VELOCITY) {
        Double newVel =
            BabyAlchemist.run(
                index,
                io.getDrivePanSalt(),
                "DrivePan Fixed",
                inputs.drivePanVelocityMetersPerSec,
                "meters/sec");
        if (newVel != null) {
          io.setDrivePanVelocity(newVel / constants.drivePanDonutRadius);
          io.setTurnPosition(new Rotation2d(0)); // drivePan straight
        }
      } else {
        BabyAlchemist.run(
            index,
            io.getDrivePanSalt(),
            "DrivePan XBox",
            inputs.drivePanVelocityMetersPerSec,
            "meters/sec");
      }
    }
  }

  public void runClosedLoopDrivePan(SwerveModuleState state) {
    if (Constants.drivePanMode == SubsystemMode.NORMAL
        || (Constants.drivePanMode == SubsystemMode.TUNING
            && Constants.drivePanTuningMode == DrivePanTuningMode.DRIVING_WITH_DRIVER)) {
      // Optimize velocity setpoint
      state.optimize(getAngle());
      state.cosineScale(getAngle());

      // Apply setpoints
      io.setDrivePanVelocity(state.speedMetersPerSecond / constants.drivePanDonutRadius);
      io.setTurnPosition(state.angle);
    }
  }

  public void runOpenLoopDrivePan(SwerveModuleState state) {
    if (Constants.drivePanMode == SubsystemMode.NORMAL
        || (Constants.drivePanMode == SubsystemMode.TUNING
            && Constants.drivePanTuningMode == DrivePanTuningMode.DRIVING_WITH_DRIVER)) {
      // Optimize velocity setpoint
      state.optimize(getAngle());
      state.cosineScale(getAngle());

      // Apply setpoints
      io.setDrivePanOpenLoop(state.speedMetersPerSecond / constants.drivePanDonutRadius);
      io.setTurnPosition(state.angle);
    }
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drivePan position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePanPositionMeters;
  }

  /** Returns the current drivePan velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.drivePanVelocityMetersPerSec;
  }

  /** Returns the module position (turn angle and drivePan position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drivePan velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }
}
