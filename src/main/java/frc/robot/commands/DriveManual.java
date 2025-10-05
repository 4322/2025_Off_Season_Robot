package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.ManualDriveMode;
import frc.robot.util.ClockUtil;

public class DriveManual extends Command {
  private Drive drive;
  private PIDController autoRotateController =
      new PIDController(Constants.Drive.autoRotatekP, 0, Constants.Drive.autoRotatekD);

  public DriveManual(Drive drive) {
    this.drive = drive;

    autoRotateController.enableContinuousInput(-Math.PI, Math.PI);
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
    ManualDriveMode driveMode = drive.getManualDriveMode();

    if (RobotContainer.getSuperstructure().getState() == Superstates.CORAL_HELD) {
      driveMode = ManualDriveMode.REEF_LOCK;
    }
    switch (driveMode) {
      case FIELD_RELATIVE:
        break;
      case AUTO_ROTATE:
        rot =
            autoRotateController.calculate(
                drive.getRotation().getRadians(), drive.getTargetAngle().getRadians());

        break;
      case REEF_LOCK:
        if (Math.abs(rot) < .01) {
          rot =
              autoRotateController.calculate(
                  drive.getRotation().getRadians(), drive.getTargetAngle().getRadians());
        }

        break;
    }
    drive.runOpenLoop(new ChassisSpeeds(dx, dy, rot), true);
  }
}
