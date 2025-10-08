package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ClockUtil;
import frc.robot.util.ReefStatus;
import org.littletonrobotics.junction.Logger;

public class DriveManual extends Command {
  private Drive drive;
  private PIDController autoRotateController =
      new PIDController(Constants.Drive.autoRotatekP, 0, Constants.Drive.autoRotatekD);
  private boolean firstReefLock;
  private double currentReefLockRad;

  private static final LoggedTunableNumber rotKp =
      new LoggedTunableNumber("AutoRotate/RotateKp", Constants.Drive.autoRotatekP);
  private static final LoggedTunableNumber rotKd =
      new LoggedTunableNumber("AutoRotate/RotateKd", Constants.Drive.autoRotatekD);

  public DriveManual(Drive drive) {
    this.drive = drive;

    autoRotateController.enableContinuousInput(-Math.PI, Math.PI);
    autoRotateController.setTolerance(Constants.Drive.angularErrorToleranceRad);
    addRequirements(drive);
  }

  @Override
  public void execute() {
    double x = -RobotContainer.driver.getLeftY();
    double y = -RobotContainer.driver.getLeftX();
    double omega =
        ClockUtil.cartesianDeadband(
            -RobotContainer.driver.getRightX(), Constants.Drive.rotDeadband);

    if (Constants.armMode == SubsystemMode.OPEN_LOOP) {
      omega = 0;
    }

    // Apply polar deadband
    double[] polarDriveCoord = ClockUtil.polarDeadband(x, y, Constants.Drive.driveDeadband);
    double driveMag = polarDriveCoord[0];
    double driveTheta = polarDriveCoord[1];

    // Quadratic scaling of drive inputs
    driveMag = driveMag * driveMag;

    // Normalize vector magnitude so as not to give an invalid input
    if (driveMag > 1) {
      driveMag = 1;
    }

    double dx = driveMag * Math.cos(driveTheta);
    double dy = driveMag * Math.sin(driveTheta);

    if (Robot.alliance == DriverStation.Alliance.Blue) {
      dx *= DrivetrainConstants.maxSpeedAt12Volts;
      dy *= DrivetrainConstants.maxSpeedAt12Volts;

    } else {
      dx *= -DrivetrainConstants.maxSpeedAt12Volts;
      dy *= -DrivetrainConstants.maxSpeedAt12Volts;
    }
    double rot = omega * omega * omega * 12.0;

    switch (drive.getManualDriveMode()) {
      case FIELD_RELATIVE:
        if (Constants.enableReefLock
            && RobotContainer.getSuperstructure().isCoralHeld()
            && Math.abs(rot) < 0.01) {
          ReefStatus reefStatus = RobotContainer.getSuperstructure().getReefStatus();
          double newReefLockRad = reefStatus.getClosestRobotAngle().getRadians();

          // Lock heading to reef face first time we engage mode
          if (!firstReefLock) {
            firstReefLock = true;
            currentReefLockRad = newReefLockRad;
          }

          if (currentReefLockRad != newReefLockRad && !reefStatus.getReefFaceAmbiguity()) {
            currentReefLockRad = newReefLockRad;
          }

          rot =
              autoRotateController.calculate(drive.getRotation().getRadians(), currentReefLockRad);
        } else if (firstReefLock) {
          firstReefLock = false;
        }
        break;
      case AUTO_ROTATE:
        if (rotKp.hasChanged(0) || rotKd.hasChanged(0)) {
          autoRotateController.setP(rotKp.get());
          autoRotateController.setD(rotKd.get());
        }
        Logger.recordOutput("AutoRotate/PIDVelocity", autoRotateController.atSetpoint());

        // Clear first reef lock if we exited field relative state while in reef lock mode
        if (firstReefLock) {
          firstReefLock = false;
        }

        rot =
            autoRotateController.calculate(
                drive.getRotation().getRadians(), drive.getTargetAngle().getRadians());
        break;
    }
    drive.runVelocity(new ChassisSpeeds(dx, dy, rot), true);
  }
}
