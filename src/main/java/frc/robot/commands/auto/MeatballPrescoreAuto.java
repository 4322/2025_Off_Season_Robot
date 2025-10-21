package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Superstates;
import frc.robot.subsystems.drive.Drive;

public class MeatballPrescoreAuto extends Command {
  private Superstructure superstructure;
  private Drive drive;
  private Rotation2d targetAngle;
  private boolean scoreBackSide;

  public MeatballPrescoreAuto(Superstructure superstructure, Drive drive) {
    this.superstructure = superstructure;
    this.drive = drive;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    if (Math.abs(drive.getRotation().minus(Rotation2d.kZero).getDegrees())
        < Math.abs(drive.getRotation().minus(Rotation2d.k180deg).getDegrees())) {
      targetAngle = Rotation2d.kZero;
      scoreBackSide = Robot.alliance == DriverStation.Alliance.Red;
    } else {
      targetAngle = Rotation2d.k180deg;
      scoreBackSide = Robot.alliance == DriverStation.Alliance.Blue;
    }
  }

  @Override
  public void execute() {
    // Only request auto rotate if not following pathplanner path command
    if (drive.getCurrentCommand() != null) {
      if (drive.getCurrentCommand() == drive.getDefaultCommand()) {
        drive.requestAutoRotateMode(targetAngle);
      }
    }

    if (Robot.alliance == Alliance.Blue) {
      if (drive.getPose().getTranslation().getX()
          >= FieldConstants.KeypointPoses.blueAutoBargePreScoreX) {
        superstructure.requestMeatballPrescore(scoreBackSide);
      }
    } else {
      if (drive.getPose().getTranslation().getX()
          <= FieldConstants.KeypointPoses.redAutoBargePreScoreX) {
        superstructure.requestMeatballPrescore(scoreBackSide);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return superstructure.armAtSetpoint()
        && superstructure.elevatorAtSetpoint()
        && superstructure.getState() == Superstates.MEATBALL_PRESCORE;
  }

  @Override
  public void end(boolean interrupted) {
    drive.requestFieldRelativeMode();
  }
}
