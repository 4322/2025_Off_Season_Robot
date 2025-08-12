package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ClockUtil;

public class DriveManual extends Command {
  private Drive drive;

  public DriveManual(Drive drive) {
    this.drive = drive;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    double x = -RobotContainer.driver.getLeftY();
    double y = -RobotContainer.driver.getLeftX();
    double omega =
        ClockUtil.cartesianDeadband(
            -RobotContainer.driver.getRightX(), Constants.Drive.rotDeadband);

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
    drive.runOpenLoop(new ChassisSpeeds(dx, dy, rot));
  }
}
